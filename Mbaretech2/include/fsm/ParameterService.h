#pragma once
#include "fsm/RuntimeParameters.h"
#include <stddef.h>

namespace fsm {
enum class ParameterSource : uint8_t { Serial, Bluetooth };
// Inicializar antes de arrancar telemetría/control. Todos los valores son RAM.
bool parameterServiceStart(const MachineRecipe& recipe);
RuntimeParameters& parameterServiceStore();
void parameterServiceRunning(bool running);
// Tras el primer comando START, el valor remoto sustituye al pin hasta reiniciar.
bool parameterServiceEffectiveStart(bool physicalStart, bool& remoteControlled);
// Cada comando START reinicia la receta en el siguiente ciclo de control.
uint32_t parameterServiceStartGeneration();
// Los transportes sólo entregan bytes o frames a una cola de espera cero.
bool parameterSubmitFrame(const char* data, size_t length);
bool parameterReceiveByte(ParameterSource source, char byte);
bool parameterIngressActive(ParameterSource source);
} // namespace fsm
