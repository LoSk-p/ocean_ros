# ocean_ros
ROS package for working with OCEAN protocol

## Requirements 

* Python 3.8.5 or later
* ROS melodic

## Install ros package

Clone ros package to your src folder in workspace:
```bash
https://github.com/LoSk-p/ocean_ros
cd ..
catkin_make
```

## Set Ethereum network & node (Rinkeby & Infura)


1. Infura runs hosted Ethereum nodes. Go to https://infura.io and sign up.
2. At Infura site, create a new project.
3. Within the project settings page, note your Infura `project id` value. We will use it in the next step.
4. Write your Infura `project id` to `network` in config file:
```bash
nano ~/catkin_ws/src/ocaen_ros/config/config.ini
```

## Set Ethereum account and get Rinkeby ETH

1. Install Metamask to your browser and generate an Etherium account. Instructions are [here](https://docs.oceanprotocol.com/tutorials/metamask-setup/).
2. Get Rinkeby ETH from a [fauset](https://faucet.rinkeby.io/).
3. [Export the private key from Metamask](https://metamask.zendesk.com/hc/en-us/articles/360015289632-How-to-Export-an-Account-Private-Key), you will need it for message.

## Install Python libraries

Create Python virtual env and install libraries:
```bash
python -m venv venv
source venv/bin/activate 
pip install ocean-lib rospkg catkin_pkg ipfshttpclient
```

## Get Rinkeby test OCEAN
Get Rinkeby OCEAN via this [fauset](https://faucet.rinkeby.oceanprotocol.com/).

## Run
```bash
roslaunch ocean_ros create_datatoken.launch
```

Publish message to `/create_datatoken` topic with the private key from your metamask account. 
Message example:
```bash
rostopic pub /create_datatoken ocean_ros/Metadata "private_key: ''                     
data_created: '2019-12-28'
type: 'dataset' 
name: 'test_ros'
author: 'author'             
license: 'CC0: Public Domain'    
files_content_type: ['text/text']
files_url: ['https://raw.githubusercontent.com/trentmc/branin/master/branin.arff']                 
tokens_nomber: 100.0
ocean_amount: 10.0"
```

