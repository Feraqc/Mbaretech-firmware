# Read-only editor server using Windows PowerShell and .NET only.
# This fallback avoids requiring Node, Python, or a working PlatformIO install.
$ErrorActionPreference = 'Stop'
$projectRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..')).TrimEnd('\', '/')
$supportFiles = @(
    'include/fsm/FSMRecipeTypes.h',
    'include/fsm/FSMDefinitions.h',
    'include/buildConfig.h',
    'include/fsm/fsm_recipe_select.h'
)
$assetFiles = @(
    'fsm_context_editor_v31.html',
    'telemetry_console.html',
    'tools/recipe-editor/core.js',
    'tools/recipe-editor/backend.js',
    'tools/recipe-editor/telemetry.js',
    'tools/recipe-editor/telemetryConsole.js',
    'tools/recipe-editor/telemetryConsole.css',
    'tools/recipe-editor/telemetryStore.js',
    'tools/recipe-editor/telemetryPresentation.js',
    'tools/recipe-editor/telemetryMock.js',
    'tools/recipe-editor/telemetryPanels.js',
    'tools/recipe-editor/telemetryWindowBridge.js',
    'tools/recipe-editor/runtimeTuning.js'
)

$port = 8765
$openBrowser = $true
for ($index = 0; $index -lt $args.Count; $index++) {
    if ($args[$index] -eq '--no-browser') {
        $openBrowser = $false
    } elseif ($args[$index] -eq '--port' -and $index + 1 -lt $args.Count -and
              $args[$index + 1] -match '^\d+$') {
        $port = [int]$args[++$index]
        if ($port -gt 65535) { throw 'Puerto fuera de rango.' }
    } else {
        throw "Opcion no reconocida: $($args[$index])"
    }
}

function Send-Response {
    param($stream, [int]$status, [byte[]]$body, [string]$contentType = 'text/plain')
    $reason = if ($status -eq 200) { 'OK' } elseif ($status -eq 501) { 'Not Implemented' } else { 'Not Found' }
    $header = "HTTP/1.1 $status $reason`r`nContent-Type: $contentType; charset=utf-8`r`nContent-Length: $($body.Length)`r`nCache-Control: no-store`r`nX-Content-Type-Options: nosniff`r`nConnection: close`r`n`r`n"
    $headerBytes = [Text.Encoding]::ASCII.GetBytes($header)
    $stream.Write($headerBytes, 0, $headerBytes.Length)
    $stream.Write($body, 0, $body.Length)
}

function Send-Text {
    param($stream, [int]$status, [string]$message)
    Send-Response $stream $status ([Text.Encoding]::UTF8.GetBytes($message))
}

function Handle-Client {
    param($client)
    $client.ReceiveTimeout = 5000
    $client.SendTimeout = 5000
    $stream = $client.GetStream()
    $reader = [IO.StreamReader]::new($stream, [Text.Encoding]::ASCII, $false, 1024, $true)
    try {
        $requestLine = $reader.ReadLine()
        if ($requestLine -notmatch '^([A-Z]+) ([^ ]+) HTTP/1\.[01]$') {
            Send-Text $stream 404 'No encontrado'
            return
        }
        $method = $Matches[1]
        $rawPath = $Matches[2]
        # Consume request headers before sending the response. Limit their count
        # so a local client cannot occupy the single-threaded fallback forever.
        for ($count = 0; $count -lt 100; $count++) {
            $line = $reader.ReadLine()
            if ($null -eq $line -or $line.Length -eq 0) { break }
        }
        if ($method -ne 'GET') {
            Send-Text $stream 501 'Metodo no admitido'
            return
        }
        try {
            $requested = [Uri]::UnescapeDataString(($rawPath -split '\?', 2)[0]).TrimStart('/')
        } catch {
            Send-Text $stream 404 'No encontrado'
            return
        }
        if ($requested.Length -eq 0) { $requested = 'fsm_context_editor_v31.html' }
        if ($requested -eq 'api/recipes') {
            $folder = Join-Path $projectRoot 'include/fsm/recipes'
            if (-not (Test-Path -LiteralPath $folder -PathType Container)) {
                Send-Text $stream 404 'No se encontro include/fsm/recipes'
                return
            }
            $names = @(Get-ChildItem -LiteralPath $folder -File | Where-Object {
                $_.Name -cmatch '^fsm_recipe_\w+\.h$'
            } | Sort-Object Name | ForEach-Object { $_.Name })
            $json = ConvertTo-Json -InputObject $names -Compress
            Send-Response $stream 200 ([Text.Encoding]::UTF8.GetBytes($json)) 'application/json'
            return
        }
        $isRecipe = $requested -cmatch '^include/fsm/recipes/fsm_recipe_\w+\.h$'
        if (($supportFiles -cnotcontains $requested) -and
            ($assetFiles -cnotcontains $requested) -and -not $isRecipe) {
            Send-Text $stream 404 'No encontrado'
            return
        }
        $target = [IO.Path]::GetFullPath((Join-Path $projectRoot $requested))
        if (-not $target.StartsWith($projectRoot + [IO.Path]::DirectorySeparatorChar,
                                     [StringComparison]::OrdinalIgnoreCase) -or
            -not [IO.File]::Exists($target)) {
            Send-Text $stream 404 'No encontrado'
            return
        }
        # Reject links in every path component. OneDrive may mark ordinary
        # project folders as reparse points, so that attribute alone is unsafe.
        $component = $target
        while ($component.Length -gt $projectRoot.Length) {
            $item = Get-Item -LiteralPath $component -Force
            if ($item.LinkType -eq 'SymbolicLink' -or $item.LinkType -eq 'Junction') {
                Send-Text $stream 404 'No encontrado'
                return
            }
            $component = [IO.Path]::GetDirectoryName($component)
        }
        $type = if ($target.EndsWith('.html')) { 'text/html' }
                elseif ($target.EndsWith('.js')) { 'text/javascript' }
                elseif ($target.EndsWith('.css')) { 'text/css' }
                else { 'text/plain' }
        Send-Response $stream 200 ([IO.File]::ReadAllBytes($target)) $type
    } finally {
        $reader.Dispose()
    }
}

$listener = [Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback, $port)
try {
    $listener.Start()
    $activePort = ([Net.IPEndPoint]$listener.LocalEndpoint).Port
    $url = "http://127.0.0.1:$activePort/fsm_context_editor_v31.html"
    Write-Host "Proyecto: $projectRoot"
    Write-Host "Editor: $url"
    Write-Host 'Ctrl+C para cerrar. El servidor no escribe archivos.'
    if ($openBrowser) {
        try { Start-Process -FilePath $url } catch { Write-Warning 'Abre manualmente la URL mostrada arriba.' }
    }
    while ($true) {
        $client = $listener.AcceptTcpClient()
        try { Handle-Client $client } catch { Write-Warning "Solicitud rechazada: $($_.Exception.Message)" }
        finally { $client.Dispose() }
    }
} finally {
    $listener.Stop()
}
