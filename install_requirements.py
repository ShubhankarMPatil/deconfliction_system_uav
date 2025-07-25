import subprocess
import sys
import pkg_resources

REQUIREMENTS_FILE = 'requirements.txt'

def check_and_install():
    # Read requirements
    with open(REQUIREMENTS_FILE) as f:
        required = [line.strip() for line in f if line.strip() and not line.startswith('#')]
    
    # Parse package names (ignore version pins for check)
    required_pkgs = [r.split('=')[0].split('>')[0].split('<')[0] for r in required]
    
    # Get installed packages
    installed = {pkg.key for pkg in pkg_resources.working_set}
    missing = [pkg for pkg in required_pkgs if pkg.lower() not in installed]
    
    if not missing:
        print('All required packages are already installed.')
        return
    print(f'Missing packages: {missing}')
    print('Installing missing packages...')
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', '-r', REQUIREMENTS_FILE])
    print('All required packages are now installed.')

if __name__ == '__main__':
    check_and_install() 