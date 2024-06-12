# Server OPC-UA

Contain OPC-UA Server, there're also 2 version:

asyncua version: It's suspected that this version run better than the python-opcua version in term of handling request, provide stable information exchange. The asyncua require python 3.7+, so we need to activate the virtual environment.

`source virtualenv/env/bin/activate`
`rosrun opc_ros server_asyncua.py`

python-opcua version: running this version with 16 mini-clients connected to Datacenter somehow make the data exchange extremely slow, with image tranmission show a delay of more than 1 minute. Also, I'm not sure that this version is able to run properly :>

`rosrun opc_ros server.py`