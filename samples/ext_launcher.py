from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

while simulation_app.is_running():
    simulation_app.update()
simulation_app.close()