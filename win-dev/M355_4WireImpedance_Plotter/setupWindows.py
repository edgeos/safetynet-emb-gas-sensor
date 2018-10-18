from distutils.core import setup
from glob import glob
import py2exe
import sys

sys.path.append(r'D:\Contracting\GE\2018_Fall\git_repo\safetynet-emb-gas-sensor\win-dev')
data_files = [("Microsoft.VC90.CRT", glob(r'D:\Contracting\GE\2018_Fall\git_repo\safetynet-emb-gas-sensor\win-dev*.*'))]

setup(
    data_files=data_files,
    windows=['simpleApp.pyw']
)