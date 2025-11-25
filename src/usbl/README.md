# USBL manager in CPython

> ⚠️ This program is a `CPython` implementation of the official USBL manager which is written in `micropython`. Keep in mind that this version could be different or/and outdated.

The purpose of this program is to act as a transparent bridge between serial and a USBL (via TCP) following latter's syntax (see more [here](manager/usbl_translator.py))

1. In a new terminal, clone the repository (or download source code):
   ```bash
   git clone https://github.com/LabMACS/USBL_Tagliatesta.git
   ```
2. Navigate to project's root directory:
    ```bash
   cd USBL_Tagliatesta
   ```
3. Create a virtual environment:
   ```bash
   python -m venv .venv
   ```
4. Activate it depending on the operating system (the path might change):
   ```bash
   source .venv/bin/activate       # Linux/MacOS
   .\.venv\Scripts\Activate.ps1    # Windows (in PowerShell)
   ```
5. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
6. Set a custom configuration in [settings](settings.py) based on your setup and hardware (e.g. serial port name and ip of tcp device)
7. Run the **CPython** implementation of the [software](main.py):
   ```bash
   python cypthon/main.py
   ```