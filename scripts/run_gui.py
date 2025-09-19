#!/usr/bin/env python3

"""
Standalone launcher for Seiretu GUI
Usage: python3 run_gui.py
"""

import sys
import os

# Add the seiretu package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# Import and run the GUI
from seiretu.seiretu_gui import main

if __name__ == '__main__':
    main()