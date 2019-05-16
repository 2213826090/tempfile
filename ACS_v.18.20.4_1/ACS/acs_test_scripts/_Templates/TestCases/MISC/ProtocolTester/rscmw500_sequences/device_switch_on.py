# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212

# Switch on the Device
self._device.switch_on()
# Confirm command
self._confirm_at_commands('\\r\\nOK\\r\\n', 'true')
