from distutils.core import setup, Extension
import os.path

UTI_sources = ["utilities_u.c"]
UTI_sourcedir="/home/nick/Documenti/Python esempi/src/utilities_u"

mod = Extension('utilities_u',
                sources = 
                [ os.path.join(UTI_sourcedir, sr) for sr in UTI_sources],
                include_dirs = [UTI_sourcedir],
                extra_compile_args = ['-std=c++0x'],
                define_macros = [("HAL_QUIET", None)]
                )

setup (name = 'utilities_uLib',
       version = '8.0.0',
       description = 'richards-tech IMU Sensor Fusion Library',
       ext_modules = [mod])
