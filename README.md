# ocean_ros
ROS package for working with OCEAN protocol

## Requirements 

* Python 3.8.5 or later
* ROS melodic

## Install ros package

Clone ros package to your src folder in workspace:
```bash
git clone https://github.com/LoSk-p/ocean_ros
cd ..
catkin_make
source devel/setup.bash
```

## Set Ethereum network & node (Rinkeby & Infura)


1. Infura runs hosted Ethereum nodes. Go to https://infura.io and sign up.
2. At Infura site, create a new project.
3. Within the project settings page, note your Infura `project id` value. We will use it in the next step.
4. Create `config.ini` file in `config` directory (example of configuration you can find in `config_template.ini` file).
5. Write your Infura `project id` to `network` in config file.

## Set Ethereum account and get Rinkeby ETH

1. Install Metamask to your browser and generate an Etherium account. Instructions are [here](https://docs.oceanprotocol.com/tutorials/metamask-setup/).
2. Get Rinkeby ETH from a [fauset](https://faucet.rinkeby.io/).
3. [Export the private key from Metamask](https://metamask.zendesk.com/hc/en-us/articles/360015289632-How-to-Export-an-Account-Private-Key), you will need it for creating and buying datatokens.

## Install Python libraries

Create Python virtual env and install libraries:
```bash
python3 -m venv venv
source venv/bin/activate 
pip install ocean-lib rospkg catkin_pkg
```

## Get Rinkeby test OCEAN
Get Rinkeby OCEAN via this [fauset](https://faucet.rinkeby.oceanprotocol.com/).

## Run
In the terminal with python virtual env run:
```bash
roslaunch ocean_ros datatokens.launch
```

### Create datatoken
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
You can see information about datatoken in `/token_response` topic:

```bash
rostopic echo /token_response
```
### Buy datatoken
Publish message to `/get_datatoken` topic with the private key from your metamask account and information about datatoken (you can find it in [market](https://market.oceanprotocol.com/) or from the `/token_response` topic). 
Message example:
```bash
rostopic pub /get_datatoken ocean_ros/BuyDatatoken "{private_key: '', destination: '/home/user/', token_address: '0x9fb21F68257F1d718d764B68b1430B6460796e42', did: 'did:op:9fb21F68257F1d718d764B68b1430B6460796e42', pool_address: '0xcF295B85ef5ADd0E513B789477C6d14eA6Bc718a'}"
```

Path to the downloaded file you can see in `/buying_response` topic:
```bash
rostopic echo /buying_response
```
