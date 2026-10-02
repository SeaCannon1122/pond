from pond import Manager
import os
import subprocess

def mujoco(pm: Manager, urdf_path: str):

    config_prefix = os.getenv("POND_IDEFIX_CONFIG_PATH", "")

    xml_path = config_prefix + "build/robot_mujoco.xml"

    # subprocess.run(
    #     ["compile", urdf_path, xml_path],
    #     input="y\n",
    #     text=True,
    #     check=True,
    # )

    # with open(xml_path, "r+", encoding="utf-8") as f:
    #     xml = f.read()
        
    #     xml = xml.replace("<worldbody>", "<worldbody>\n  <body name=\"chassis\" pos=\"0 0 0.3\">\n    <freejoint/>", 1)
    #     xml = xml.replace("</worldbody>", "  </body>\n    </worldbody>", 1)
    #     xml = xml.replace("</mujoco>", "<include file=\"" + config_prefix + "mujoco.xml\"/>" + "\n</mujoco>", 1)

    #     f.seek(0)
    #     f.write(xml)
    #     f.truncate()
    
    pm.load_module(
        "simulator", "mujoco/mujoco", "simulation_thread",
        {
            "robot_path" : xml_path,
            "actuator_groups" : [
                #{"names" : drive.cfg.motor_names, "types" : len(drive.cfg.motor_names)*["velocity"]},
            ]
        },
    )