import os
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False}, experience=f'{os.environ["EXP_PATH"]}/isaacsim.exp.full.kit')

while simulation_app.is_running():
    simulation_app.update()
simulation_app.close()