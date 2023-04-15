# RsPython

Generic Python code of the Robotic Sensing Lab

To use the code, add the path of rsPython to the system path and import libraries:
e.g.
import sys
sys.path.append('..\\rsPython')
import rsImaging.Perspective as Perspective

type hints:
https://mypy.readthedocs.io/en/stable/cheat_sheet_py3.html
python 3.9 nodig voor list[Motors.Motor]  dict[str:Number]

Anaconda users can easily create a new environment from `openAIgym.yml`. Navigate to your *rsPython* folder and run following commands in an Anaconda prompt:  
`conda create -f openAIgym.yml`  
`conda activate openAIgym`  
`conda develop .`
