/* Adaptación del grafo visual v31 al API actual, sin cambiar su representación. */
(function (root) {
  'use strict';
  const C = typeof module !== 'undefined' ? require('./core.js') : root.RecipeCore;

  function triggerToRecipe(trigger, api) {
    if (!trigger) throw new Error('La transición no tiene condición.');
    const types = {timer: 'TIMER', macro: 'SENSOR', completion: 'COMPLETION'};
    if (!types[trigger.type]) throw new Error('Condición del grafo incompatible.');
    return {
      type: types[trigger.type],
      timer: trigger.firmwareTimer !== undefined && C.number(api, trigger.firmwareTimer) === trigger.duration_ms
        ? trigger.firmwareTimer : (trigger.duration_ms ?? 0),
      timerParameter: trigger.timerParameter || 0,
      terms: [...(trigger.terms || [])],
      operators: [...(trigger.operators || [])]
    };
  }

  function fromGraph(api, graph, identity) {
    if (!api) throw new Error('Carga FSMRecipeTypes.h y FSMDefinitions.h antes de exportar.');
    if (graph.nodes.some(node => !/^S\d+$/.test(node.id)) || graph.edges.some(edge => !/^T\d+$/.test(edge.id))) throw new Error('Identificadores del grafo incompatibles.');
    if (new Set(graph.edges.map(edge=>edge.id)).size !== graph.edges.length) throw new Error('Identificadores de transiciones duplicados.');
    const initial = graph.nodes.filter(node => node.initial);
    if (initial.length !== 1) throw new Error('Debe existir exactamente un estado inicial.');
    const nodes = new Map(graph.nodes.map(node => [node.id, node]));
    if (nodes.size !== graph.nodes.length) throw new Error('Hay identificadores de nodos duplicados.');
    const stateIds = new Set(), stateNames = new Set();
    for (const node of graph.nodes) {
      if (stateIds.has(node.stateId)) throw new Error('StateId duplicado: ' + node.stateId + '.');
      stateIds.add(node.stateId);
      const name = (node.name || '').trim();
      if (!name) throw new Error('El estado ' + node.stateId + ' necesita un nombre.');
      const key = name.toLocaleLowerCase();
      if (stateNames.has(key)) throw new Error('Nombre de estado duplicado: ' + name + '.');
      stateNames.add(key);
    }
    for (const edge of graph.edges) {
      if (!nodes.has(edge.from) || !nodes.has(edge.to)) throw new Error('Una transición tiene origen o destino inexistente.');
    }
    // Conservar referencias centrales mientras el control visual no cambie su valor.
    const retained = (value, original) => original !== undefined && C.number(api, original) === value ? original : value;
    const command = item => ({
      motion: item.state,
      left: retained(item.params?.left_speed_pct, item.firmwareCommand?.left),
      right: retained(item.params?.right_speed_pct, item.firmwareCommand?.right),
      leftParameter:item.params?.leftParameter||0,
      rightParameter:item.params?.rightParameter||0
    });
    const recipe = {
      name: graph.name,
      ...(identity || C.normalize(graph.name)),
      initialState: initial[0].stateId,
      parameters:(graph.parameters||[]).map(parameter=>({...parameter})),
      states: graph.nodes.map(node => {
        if (!['base', 'subfsm'].includes(node.type)) throw new Error('Tipo de nodo incompatible.');
        const steps = node.sequence || [];
        if (steps.some(step => !/^P\d+$/.test(step.id) || !Array.isArray(step.out_transitions) || step.out_transitions.some(edge => !/^ST\d+$/.test(edge.id)))) throw new Error('Identificadores de pasos incompatibles.');
        if (new Set(steps.map(step => step.id)).size !== steps.length) throw new Error('Identificadores de pasos duplicados.');
        return {
          id: node.stateId,
          kind: node.type === 'subfsm' ? 'SUBFSM' : 'MOTOR',
          ...command(node),
          hold: node.allowHoldOnCompletion === true,
          transitions: graph.edges.filter(edge => edge.from === node.id).map(edge => ({
            ...triggerToRecipe(edge.trigger, api), target: nodes.get(edge.to).stateId
          })),
          steps: steps.map(step => ({
            ...command(step),
            transitions: step.out_transitions.map(edge => {
              const target = edge.next_step === 'COMPLETE' ? -1 : steps.findIndex(candidate => candidate.id === edge.next_step);
              if (target === -1 && edge.next_step !== 'COMPLETE') throw new Error('El destino del paso no existe.');
              return {...triggerToRecipe(edge.trigger, api), target};
            })
          }))
        };
      })
    };
    const errors = C.validate(api, recipe);
    if (errors.length) throw new Error('No se puede generar la receta.\n• ' + errors.join('\n• '));
    return recipe;
  }

  function toGraph(api, recipe) {
    const errors = C.validate(api, recipe);
    if (errors.length) throw new Error(errors.join('\n'));
    let nextEdge = 1, nextStep = 1, nextStepTransition = 1;
    const ids = new Map(recipe.states.map((state, index) => [state.id, 'S' + (index + 1)]));
    const timerNames = new Set();
    const timerMacro = label => {
      const base = (String(label).toUpperCase().replace(/[^A-Z0-9_]/g, '_').replace(/_+/g, '_').replace(/^_+|_+$/g, '') || 'STATE') + '_TIMER';
      let name = base, suffix = 2;
      while (timerNames.has(name)) name = base + '_' + suffix++;
      timerNames.add(name);
      return name;
    };
    const trigger = (item, label) => ({
      type: {TIMER:'timer', SENSOR:'macro', COMPLETION:'completion'}[item.type],
      duration_ms: C.number(api, item.timer), firmwareTimer: item.timer,
      timerParameter:item.timerParameter||0,
      // A recipe stores the timer value, not its editor label. Give each
      // imported timer a distinct, readable label without changing that value.
      macro: item.type === 'TIMER' ? timerMacro(label) : undefined,
      terms: [...item.terms], operators: [...item.operators]
    });
    const params = item => ({left_speed_pct: C.number(api, item.left), right_speed_pct: C.number(api, item.right),
      leftParameter:item.leftParameter||0,rightParameter:item.rightParameter||0});
    const edges = [];
    const nodes = recipe.states.map((state, index) => {
      const stepIds = state.steps.map(() => 'P' + nextStep++);
      const stateName = api.metadata[state.id]?.name || state.id;
      const node = {
        // StateId is the firmware key; the catalog name is the editable label.
        id: ids.get(state.id), stateId: state.id, name: stateName,
        type: state.kind === 'SUBFSM' ? 'subfsm' : 'base', state: state.motion,
        params: params(state), firmwareCommand: {left:state.left, right:state.right}, initial: state.id === recipe.initialState,
        context: api.metadata[state.id]?.description || '',
        x: 100 + (index % 3) * 260, y: 100 + Math.floor(index / 3) * 160,
        w: state.kind === 'SUBFSM' ? 190 : 165, h: state.kind === 'SUBFSM' ? 72 : 66,
        allowHoldOnCompletion: state.hold,
        sequence: state.steps.map((step, i) => ({
          id: stepIds[i], state: step.motion, params: params(step),
          firmwareCommand: {left:step.left, right:step.right},
          out_transitions: step.transitions.map(edge => ({
            id: 'ST' + nextStepTransition++, trigger: trigger(edge, stateName + '_' + step.motion),
            next_step: edge.target === -1 ? 'COMPLETE' : stepIds[edge.target]
          }))
        }))
      };
      for (const edge of state.transitions) edges.push({
        id: 'T' + nextEdge++, from: node.id, to: ids.get(edge.target), trigger: trigger(edge, stateName), action: ''
      });
      return node;
    });
    return {name: recipe.name, parameters:(recipe.parameters||[]).map(parameter=>({...parameter})),
      nodes, edges, nextNode: nodes.length + 1, nextEdge, nextStep, nextStepTransition};
  }

  const paths = {
    types: 'include/fsm/FSMRecipeTypes.h', definitions: 'include/fsm/FSMDefinitions.h',
    config: 'include/buildConfig.h', selection: 'include/fsm/fsm_recipe_select.h'
  };

  // Registrar identidades nuevas en el catálogo central, nunca dentro de la receta.
  function appendStateDefinition(source, baseId, stateId) {
    if (!/^[A-Za-z_]\w*$/.test(baseId) || !/^[A-Za-z_]\w*$/.test(stateId)) throw new Error('StateId incompatible.');
    const enumPattern = /enum\s+class\s+StateId\s*:\s*uint8_t\s*\{([^}]*)\}/;
    const enumeration = source.match(enumPattern);
    if (!enumeration || !/\bCOUNT\s*$/.test(enumeration[1])) throw new Error('No se puede ampliar StateId: se requiere COUNT al final.');
    const metadataPattern = /(static\s+constexpr\s+StateMetadata\s+STATES\s*\[\s*\]\s*=\s*\{)([\s\S]*?)(\r?\n\};)/;
    const metadata = source.match(metadataPattern);
    if (!metadata) throw new Error('No se encontró el catálogo STATES en FSMDefinitions.h.');
    const newline = source.includes('\r\n') ? '\r\n' : '\n';
    const withEnum = source.replace(enumPattern, full => full.replace(/\bCOUNT\s*\}/, stateId + ', COUNT }'));
    const withMetadata = withEnum.replace(metadataPattern, (_, start, body, end) => {
      const previous = body.trimEnd();
      return start + previous + (previous.endsWith(',') ? '' : ',') + newline +
        '    {StateId::' + stateId + ', "' + stateId + '", "Instancia de ' + baseId + '"}' + end;
    });
    const namesPattern = /(static\s+constexpr\s+const\s+char\*\s+STATE_ID_NAMES\s*\[\s*\]\s*=\s*\{)([\s\S]*?)(\s*\};)/;
    if (!namesPattern.test(withMetadata)) throw new Error('Falta STATE_ID_NAMES en FSMDefinitions.h.');
    return withMetadata.replace(namesPattern, (_, start, body, end) =>
      start + body.trimEnd() + (body.trimEnd().endsWith(',') ? '' : ',') + newline +
      '    "' + stateId + '"' + end);
  }

  function setStateMetadataName(source, stateId, name) {
    const escaped = stateId.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
    const entry = new RegExp('(\\{\\s*StateId::' + escaped + '\\s*,\\s*)"(?:\\\\.|[^"\\\\])*"');
    if (!entry.test(source)) throw new Error('Falta StateMetadata para ' + stateId + '.');
    return source.replace(entry, (_, prefix) => prefix + JSON.stringify(name));
  }

  function stateIdFromName(name) {
    const id = String(name || '').trim().normalize('NFKD')
      .replace(/[\u0300-\u036f]/g, '').toUpperCase()
      .replace(/[^A-Z0-9]+/g, '_').replace(/^_+|_+$/g, '');
    if (!id || /^[0-9]/.test(id) || id === 'COUNT')
      throw new Error('Nombre de estado incompatible con StateId: ' + name + '.');
    return id;
  }

  function exportWithStateNames(project, graph, identity) {
    // El export transforma una copia: las referencias del editor y del robot
    // conectado no cambian hasta importar/flashear los archivos generados.
    const named = structuredClone(graph);
    const replacements = named.nodes.map(node => ({node, oldId:node.stateId,
      newId:stateIdFromName(node.name)}));
    const ids = replacements.map(item => item.newId);
    if (new Set(ids).size !== ids.length)
      throw new Error('Dos nombres producen el mismo StateId. Renombra uno antes de exportar.');
    for (const item of replacements)
      if (item.newId !== item.oldId && replacements.some(other =>
          other !== item && other.oldId === item.newId))
        throw new Error('El StateId ' + item.newId + ' pertenece a otro estado de esta receta.');
    let definitions = project.definitionsSource;
    // Liberar nombres de metadatos antiguos antes de añadir los nuevos tokens.
    for (const item of replacements)
      if (item.newId !== item.oldId)
        definitions = setStateMetadataName(definitions, item.oldId, item.oldId);
    let api = C.loadAPI(project.files.get(paths.types), definitions);
    for (const item of replacements) {
      if (!api.states.includes(item.newId)) {
        if (api.states.length >= 255) throw new Error('El catálogo uint8_t no admite más estados.');
        definitions = appendStateDefinition(definitions, item.oldId, item.newId);
      }
      definitions = setStateMetadataName(definitions, item.newId, item.node.name);
      api = C.loadAPI(project.files.get(paths.types), definitions);
      item.node.stateId = item.newId;
    }
    return {recipe:fromGraph(api, named, identity), definitions, api};
  }

  class Project {
    constructor() {
      this.api = null;
      this.files = new Map();
      this.directory = null;
      this.recipes = [];
      this.errors = [];
      this.unavailable = [];
      this.options = {};
      this.pending = null;
      this.instanceDefinitions = [];
      this.definitionsSource = null;
    }

    load(files) {
      for (const path of Object.values(paths)) {
        if (!files.has(path)) throw new Error('No se encontró: ' + path);
      }
      const api = C.loadAPI(files.get(paths.types), files.get(paths.definitions));
      const recipes = [], errors = [], unavailable = [], entries = C.selections(files.get(paths.selection));
      for (const [path, source] of files) {
        if (!path.startsWith('include/fsm/recipes/')) continue;
        const file = path.split('/').pop();
        // Un placeholder explícito no es una receta mal formada: sigue en el
        // firmware, pero el editor no debe ofrecerlo como receta editable.
        const marker = source.match(/^\/\/ FSM_RECIPE_UNAVAILABLE:\s*(.+)$/m);
        if (marker) {
          unavailable.push(file + ': ' + marker[1]);
          continue;
        }
        try {
          const recipe = C.importRecipe(api, source, file);
          const entry = entries.find(item => item.file === file);
          if (entry) recipe.macro = entry.macro;
          recipes.push(recipe);
        } catch (error) { errors.push(file + ': ' + error.message); }
      }
      this.api = api;
      this.files = new Map(files);
      this.recipes = recipes;
      this.errors = errors;
      this.unavailable = unavailable;
      this.options = C.configValues(files.get(paths.config));
      this.definitionsSource = files.get(paths.definitions);
      this.instanceDefinitions = [];
      this.pending = null;
      return this;
    }

    allocateStateId(baseId, usedIds) {
      if (!this.api?.states.includes(baseId)) throw new Error('StateId base desconocido: ' + baseId);
      const used = new Set(usedIds);
      if (!used.has(baseId)) return baseId;
      let suffix = 1;
      while (used.has(baseId + '_' + suffix)) suffix++;
      const stateId = baseId + '_' + suffix;
      if (!this.api.states.includes(stateId)) {
        if (this.api.states.length >= 255) throw new Error('El catálogo uint8_t no admite más estados.');
        const source = appendStateDefinition(this.definitionsSource, baseId, stateId);
        const api = C.loadAPI(this.files.get(paths.types), source);
        this.definitionsSource = source;
        this.api = api;
        this.instanceDefinitions.push({baseId, stateId});
        this.pending = null;
      }
      return stateId;
    }

    restoreGraph(graph, identity, instances = []) {
      // Validar todos los datos antes de modificar el proyecto en memoria.
      if (!Array.isArray(instances) || instances.length > 255) throw new Error('Catálogo de instancias incompatible.');
      let source = this.definitionsSource;
      let api = this.api;
      const additions = [...this.instanceDefinitions];
      for (const item of instances) {
        if (!item || !api.states.includes(item.baseId) || !/^.+_[1-9]\d*$/.test(item.stateId) || !item.stateId.startsWith(item.baseId + '_')) throw new Error('Identidad de instancia incompatible.');
        if (api.states.includes(item.stateId)) continue;
        if (api.states.length >= 255) throw new Error('El catálogo uint8_t no admite más estados.');
        source = appendStateDefinition(source, item.baseId, item.stateId);
        api = C.loadAPI(this.files.get(paths.types), source);
        additions.push({baseId:item.baseId, stateId:item.stateId});
      }
      const recipe = fromGraph(api, graph, identity);
      this.api = api;
      this.definitionsSource = source;
      this.instanceDefinitions = additions;
      this.pending = null;
      return recipe;
    }

    definitionsChanged() {
      return this.definitionsSource !== this.files.get(paths.definitions);
    }

    definitionsWithNames(nodes) {
      // Names live in the shared catalog, not in MachineRecipe. Patch only the
      // metadata entries for states present in this graph and retain their IDs.
      let source = this.definitionsSource;
      const names = new Set();
      for (const node of nodes) {
        const name = (node.name || '').trim();
        if (!name) throw new Error('El estado ' + node.stateId + ' necesita un nombre.');
        const key = name.toLocaleLowerCase();
        if (names.has(key)) throw new Error('Nombre de estado duplicado: ' + name + '.');
        names.add(key);
        if (!this.api.states.includes(node.stateId)) throw new Error('StateId desconocido: ' + node.stateId);
        const escapedId = node.stateId.replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
        const entry = new RegExp('(\\{\\s*StateId::' + escapedId + '\\s*,\\s*)"(?:\\\\.|[^"\\\\])*"');
        if (!entry.test(source)) throw new Error('Falta StateMetadata para ' + node.stateId + '.');
        source = source.replace(entry, (_, prefix) => prefix + JSON.stringify(name));
      }
      // Include states outside the current recipe when checking catalog names.
      C.loadAPI(this.files.get(paths.types), source);
      return source;
    }

    async openFromUrl(baseUrl, fetchFile = globalThis.fetch) {
      const rootUrl = new URL('.', baseUrl);
      if (!['http:', 'https:'].includes(rootUrl.protocol)) {
        throw new Error('Abre iniciar_editor.cmd dentro de Mbaretech2. El navegador no puede leer headers desde file://.');
      }
      const read = async path => {
        const response = await fetchFile(new URL(path, rootUrl).href, {cache:'no-store'});
        if (!response.ok) throw new Error('No se encontró: ' + path + ' (HTTP ' + response.status + ')');
        return response.text();
      };
      // Rutas fijas del proyecto; los contenidos siempre se leen del firmware real.
      const entries = await Promise.all(Object.values(paths).map(async path => [path, await read(path)]));
      const files = new Map(entries);
      const names = JSON.parse(await read('api/recipes'));
      if (!Array.isArray(names) || names.some(name => typeof name !== 'string' || !/^fsm_recipe_\w+\.h$/.test(name))) {
        throw new Error('Listado de recetas incompatible.');
      }
      await Promise.all(names.map(async name => {
        const path = 'include/fsm/recipes/' + name;
        files.set(path, await read(path));
      }));
      const next = new Project();
      next.load(files);
      Object.assign(this, next);
      return this;
    }

    async fileHandle(path, create = false) {
      let handle = this.directory;
      const parts = path.split('/');
      for (const part of parts.slice(0, -1)) handle = await handle.getDirectoryHandle(part, {create});
      return handle.getFileHandle(parts.at(-1), {create});
    }

    async open(directory) {
      const next = new Project();
      next.directory = directory;
      const files = new Map();
      for (const path of Object.values(paths)) {
        try { files.set(path, await (await (await next.fileHandle(path)).getFile()).text()); }
        catch { throw new Error('No se encontró: ' + path); }
      }
      let folder = directory;
      for (const part of ['include', 'fsm', 'recipes']) {
        try { folder = await folder.getDirectoryHandle(part); }
        catch { throw new Error('No se encontró: include/fsm/recipes/'); }
      }
      for await (const [name, handle] of folder.entries()) {
        if (handle.kind === 'file' && /^fsm_recipe_\w+\.h$/.test(name)) files.set('include/fsm/recipes/' + name, await (await handle.getFile()).text());
      }
      next.load(files);
      Object.assign(this, next);
      return this;
    }

    configure(changes) {
      for (const [key, value] of Object.entries(changes)) {
        if (!(key in this.options) || ![0, 1].includes(value)) throw new Error('Opción 0/1 desconocida: ' + key);
      }
      this.options = {...this.options, ...changes};
      this.pending = null;
    }

    preview(recipe, graph = null) {
      const v = this.options;
      const errors = [];
      const need = (condition, requirement, text) => { if (condition && !requirement) errors.push(text); };
      const programs = ['ENABLE_FSM','ENABLE_RECIPE_FSM','ENABLE_GYRO_TEST','ENABLE_MOVEMENT_TEST','ENABLE_MOTOR_TEST','ENABLE_LINE_TEST','ENABLE_TURN_CALIBRATION'];
      if (programs.reduce((sum,key)=>sum+(v[key]||0),0)>1) errors.push('Solo puede haber un programa activo.');
      need(v.ENABLE_RECIPE_FSM,v.ENABLE_SENSOR_TASK,'Recipe FSM requiere la tarea de sensores.');
      need(v.ENABLE_WIFI_TELEMETRY,v.ENABLE_TELEMETRY && (v.ENABLE_RECIPE_FSM||v.ENABLE_FSM),'Telemetría WiFi requiere telemetría y Recipe FSM o combate.');
      need(v.ENABLE_TELEMETRY,v.ENABLE_SERIAL||v.ENABLE_BLE||v.ENABLE_WIFI_TELEMETRY,'Telemetría requiere un transporte.');
      need(v.ENABLE_TELEMETRY,v.ENABLE_RECIPE_FSM||v.ENABLE_FSM,'Telemetría requiere Recipe FSM o combate.');
      need(v.ENABLE_RECIPE_FSM,v.ENABLE_SERIAL||v.ENABLE_LOGGING,'Recipe FSM requiere Serial o Logging.');
      need(v.ENABLE_FSM,v.ENABLE_SENSOR_TASK&&v.ENABLE_LINE_SENSORS&&v.ENABLE_IR_SENSORS,'FSM requiere sensores, línea e IR.');
      need(v.ENABLE_BLE,v.ENABLE_LOGGING,'BLE requiere Logging.');
      need(v.ENABLE_LOGGING,v.ENABLE_SERIAL||v.ENABLE_BLE,'Logging requiere transporte.');
      need(v.ENABLE_DEBUG,v.ENABLE_SERIAL,'Depuración requiere Serial.');
      need(v.ENABLE_TASK_TIMING,v.ENABLE_SENSOR_TASK,'Medición de tiempos requiere sensores.');
      need(v.ENABLE_GYRO_TEST,v.ENABLE_GYRO&&v.ENABLE_SERIAL&&!v.ENABLE_LOGGING,'Gyro requiere IMU y Serial, sin Logging.');
      need(v.ENABLE_MOTOR_TEST,v.ENABLE_MOTORS&&v.ENABLE_SERIAL,'Prueba de motores requiere motores y Serial.');
      need(v.ENABLE_LINE_TEST,v.ENABLE_LINE_SENSORS&&v.ENABLE_SERIAL,'Prueba de línea requiere línea y Serial.');
      need(v.ENABLE_MOVEMENT_TEST,v.ENABLE_SERIAL&&v.ENABLE_LINE_SENSORS&&v.ENABLE_IR_SENSORS&&v.ENABLE_DIP_SWITCHES,'Movimientos requiere Serial, línea, IR y DIP.');
      need(v.ENABLE_SENSOR_TASK,!(v.ENABLE_MOVEMENT_TEST||v.ENABLE_LINE_TEST||v.ENABLE_TURN_CALIBRATION),'Los diagnósticos de lectura directa excluyen la tarea de sensores.');
      need(v.ENABLE_LEGACY_MOVEMENTS,v.ENABLE_MOVEMENT_TEST,'Movimientos existentes requiere su programa.');
      need(v.ENABLE_TURN_CALIBRATION,false,'Calibración antigua no soportada.');
      if (errors.length) throw new Error(errors.join('\n'));
      const header = C.emit(this.api, recipe);
      const original = this.files.get(paths.selection);
      const entries = C.selections(original);
      if (entries.some(entry => entry.macro === recipe.macro && entry.file !== recipe.file)) throw new Error('El selector ya identifica otra receta.');
      const known = entries.find(entry => entry.file === recipe.file);
      if (known && (known.namespace !== recipe.namespace || known.macro !== recipe.macro)) throw new Error('La identidad no coincide con la selección existente.');
      if (!known) entries.push(recipe);
      const planned = new Map([
        ['include/fsm/recipes/' + recipe.file, header],
        [paths.config, C.patchConfig(this.files.get(paths.config), this.options, recipe.macro)],
        [paths.selection, C.patchSelection(original, entries)]
      ]);
      const definitions = graph ? this.definitionsWithNames(graph.nodes) : this.definitionsSource;
      if (definitions !== this.files.get(paths.definitions)) planned.set(paths.definitions, definitions);
      this.pending = [...planned].filter(([path, after]) => after !== this.files.get(path))
        .map(([path, after]) => ({path, before: this.files.get(path), after}));
      return structuredClone(this.pending);
    }

    async saveReviewed() {
      if (!this.directory || !this.pending) throw new Error('Primero abre una carpeta y revisa previewSave().');
      if (await this.directory.requestPermission({mode:'readwrite'}) !== 'granted') throw new Error('Permiso de escritura no concedido.');
      // Comparar también el API y la configuración para detectar ediciones externas.
      const checks = new Map(Object.values(paths).map(path => [path, this.files.get(path)]));
      for (const item of this.pending) checks.set(item.path, item.before);
      for (const [path, before] of checks) {
        let actual;
        try { actual = await (await (await this.fileHandle(path)).getFile()).text(); }
        catch (error) { if (error.name !== 'NotFoundError') throw error; }
        if (actual !== before) throw new Error('Cambio externo en ' + path + '. Recarga antes de guardar.');
      }
      const done = [];
      try {
        for (const item of this.pending) {
          const writer = await (await this.fileHandle(item.path, true)).createWritable();
          await writer.write(item.after);
          await writer.close();
          done.push(item.path);
          this.files.set(item.path, item.after);
        }
      } catch (error) {
        throw new Error(error.message + '\nArchivos ya guardados: ' + done.join(', '));
      } finally { this.pending = null; }
      this.load(this.files);
      return done;
    }
  }

  const exported = {fromGraph, toGraph, exportWithStateNames, Project, paths};
  if (typeof module !== 'undefined') module.exports = exported;
  else root.RecipeBackend = exported;
})(globalThis);
