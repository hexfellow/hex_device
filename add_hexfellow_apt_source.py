#!/usr/bin/env python3

import os
import sys
import subprocess
import argparse
from pathlib import Path
from typing import Optional

def check_root_privileges():
    """Check if the script is running with root privileges."""
    if os.geteuid() != 0:
        print("Error: This script requires root privileges. Please run with sudo.", file=sys.stderr)
        sys.exit(1)

def add_hexfellow_apt_source(force: bool = False) -> None:
    """
    Add Hexfellow APT source
    
    Args:
        force: Whether to force re-adding the source, will remove existing sources first
    """
    # Color output helper function
    def color_print(message: str, color: str) -> None:
        colors = {
            'red': '\033[91m',
            'green': '\033[92m',
            'yellow': '\033[93m',
            'blue': '\033[94m',
            'reset': '\033[0m'
        }
        print(f"{colors.get(color, '')}{message}{colors['reset']}")
    
    # Execute command and show output
    def run_command_live(cmd: str) -> bool:
        color_print(f"running-command: {cmd}", "green")
        process = subprocess.Popen(
            cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )
        
        for line in process.stdout:
            print(line.strip())
        
        return process.wait() == 0
    
    # Check and remove hexmove apt source
    hexmove_source_path = "/etc/apt/sources.list.d/hexmove.sources"
    if os.path.exists(hexmove_source_path):
        color_print("Removing legacy apt source", "yellow")
        
        # Remove hexmove.sources file
        if not run_command_live("sudo rm -f /etc/apt/sources.list.d/hexmove.sources"):
            raise RuntimeError("Failed to remove hexmove.sources")
        
        # Remove hexmove-rosdep-meta package
        if not run_command_live("sudo apt-get remove -y hexmove-rosdep-meta"):
            raise RuntimeError("Failed to remove hexmove-rosdep-meta")
    
    # Force mode handling
    if force:
        color_print("Force adding hexfellow apt source", "yellow")
        
        # Remove all possible hexfellow source files
        files_to_remove = [
            "/etc/apt/sources.list.d/hexfellow.sources",
            "/etc/apt/sources.list.d/hexfellow.list",
            "/etc/apt/sources.list.d/hexfellow.lists"
        ]
        
        for file_path in files_to_remove:
            if os.path.exists(file_path):
                if not run_command_live(f"sudo rm -f {file_path}"):
                    raise RuntimeError(f"Failed to remove {file_path}")
    
    # Check if hexfellow apt source is already added
    hexfellow_source_path = "/etc/apt/sources.list.d/hexfellow.sources"
    if os.path.exists(hexfellow_source_path):
        color_print("HexFellow apt source already exists", "green")
        return
    
    color_print("Adding hexfellow apt source", "green")
    
    # Get Ubuntu version codename
    def get_ubuntu_codename() -> str:
        try:
            result = subprocess.run(
                ["lsb_release", "-cs"],
                capture_output=True,
                text=True,
                check=True
            )
            if result.stdout.strip() == "focal":
                return "focal"
            elif result.stdout.strip() == "jammy":
                return "jammy"
            elif result.stdout.strip() == "noble":
                return "noble"
            else:
                raise RuntimeError("Invalid ubuntu version")
        except (subprocess.CalledProcessError, FileNotFoundError):
            raise RuntimeError("Failed to get ubuntu version")
    
    try:
        ubuntu_codename = get_ubuntu_codename()
    except RuntimeError as e:
        color_print(str(e), "red")
        sys.exit(1)
    
    # Download GPG key
    if not run_command_live("sudo wget -q -O /usr/share/keyrings/hexfellow.gpg.key https://apt.hexfellow.com/conf/hexfellow.gpg.key"):
        raise RuntimeError("Failed to download hexfellow.gpg.key")
    
    # Create apt source config file
    apt_content = f"""Types: deb
URIs: https://apt.hexfellow.com/hexfellow/
Suites: {ubuntu_codename}
Components: main
Signed-By: /usr/share/keyrings/hexfellow.gpg.key
"""
    
    # Write to temporary file
    apt_source_path = Path("/tmp/hexfellow.sources")
    try:
        apt_source_path.write_text(apt_content)
    except Exception as e:
        raise RuntimeError("Failed to write apt source file")
    
    try:
        # Move to the correct location
        if not run_command_live(f"sudo mv {apt_source_path} {hexfellow_source_path}"):
            raise RuntimeError("Failed to move hexfellow.sources")
        
        # Update APT cache
        if not run_command_live("sudo apt-get update"):
            raise RuntimeError("Failed to update apt cache")
        
        color_print("HexFellow apt source added successfully.", "green")
        
        # Install hexfellow-rosdep-meta package
        if not run_command_live("sudo apt-get install -y hexfellow-rosdep-meta"):
            raise RuntimeError("Failed to install hexfellow-rosdep-meta")
            
    except Exception as e:
        # Clean up temporary file
        if os.path.exists(apt_source_path):
            os.unlink(apt_source_path)
        raise RuntimeError(f"Error adding apt source: {str(e)}")

# Usage example
if __name__ == "__main__":
    check_root_privileges()
    parser = argparse.ArgumentParser(description='Add Hexfellow APT source')
    parser.add_argument(
        '--force', '-f', 
        action='store_true', 
        help='Force re-adding the source, will remove existing sources first'
    )
    args = parser.parse_args()
    try:
        add_hexfellow_apt_source(force=args.force)
        
    except Exception as e:
        print(f"Error: {str(e)}", file=sys.stderr)
        sys.exit(1)
