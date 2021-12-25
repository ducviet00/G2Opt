# G2OPT

## INSTALLATION

~~~bash
pip install -r requirements.txt
~~~

## RUN

~~~bash
python Simulate.py
~~~

options:

- experiment_type: 'node', 'target', 'MC', 'prob', 'package' or 'cluster'.
- experiment_index: 0 - 4

## NOTE

- 2-opt algorithm takes long time to run, set `self.two_opt = 0` in `Optimizer/G2OPT.py` to disable 2opt
