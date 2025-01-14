1. Create a virtual enviroment
```
python3 -m venv myenv
```
2. Add this or simialr to your ~/.bashrc file
export PYTHONPATH="/home/asaace00/repo/winter/project/TAMIR/tamir/myenv/lib/python3.12/site-packages:$PYTHONPATH"

basically, you just need a reference to your virtual enviroment's python3.12-site-packages attached to your PYTHONPATH

3. Source your enviroment
source myenv/bin/activate

4. install packages
pip3 install -r requirements.txt