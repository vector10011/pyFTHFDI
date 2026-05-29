# MacOS preparation
1. Install pyenv and Python 3.12.13
```
brew install pyenv
pyenv install 3.12.13
```
2. Set the local Python version for the project
```
cd /path/to/your/project
pyenv local 3.12.13
python --version
```
3. Create a virtual environment for the project:
```
python -m venv .venv
source .venv/bin/activate
python --version
which python
```
4. Install the required packages:
```
...
pip install import-ipynb
pip install cvxpy
```