# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212

# Switch off the Device
self._device.switch_off()
# Confirm command
self._confirm_at_commands('\\r\\nOK\\r\\n', 'true')
