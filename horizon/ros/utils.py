import subprocess

def roslaunch(package, launch):
    """
    Run a roslaunch in a separate thread
    Args:
        package: where the roslaunch is located
        launch: file
    """
    cmd = f'roslaunch {package} {launch}'.split()
    subprocess.Popen(cmd)

