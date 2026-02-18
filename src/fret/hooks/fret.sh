# Set PYTHONPATH to include fret Python modules
# Allows launch files and other code to import fret.launch.model and other packages
# COLCON_CURRENT_PREFIX is set by colcon when sourcing this hook
export PYTHONPATH="${COLCON_CURRENT_PREFIX}/share:${PYTHONPATH}"
