#!/usr/bin/env python3
import rospy
from ocean_lib.config import Config
from ocean_lib.ocean.ocean import Ocean
from ocean_lib.web3_internal.wallet import Wallet
from ocean_lib.data_provider.data_service_provider import DataServiceProvider
from ocean_lib.common.agreements.service_factory import ServiceDescriptor
from ocean_lib.models.btoken import BToken
from ocean_lib.common.agreements.service_types import ServiceTypes
from ocean_ros.msg import TokenResponse
from ocean_ros.msg import Metadata
from ocean_ros.msg import CreatingPool
import time
import os

rospy.init_node("pool_creator", anonymous=False)
config_path = rospy.get_param("~config")

pub_response = rospy.Publisher("token_response", TokenResponse, queue_size=10)

config = Config(config_path)
ocean = Ocean(config)
rospy.loginfo('pool_creator is ready')

def create_pool_callback(data):
    rospy.loginfo(f'Got create pool request')
    with open(data.private_key_path, 'r') as f:
        private_key = f.read().rstrip()
    wallet = Wallet(ocean.web3, private_key=private_key)
    OCEAN_token = BToken(ocean.OCEAN_address)
    assert OCEAN_token.balanceOf(wallet.address) > 0, "need OCEAN"
    rospy.loginfo(f'Creating pool')
    time.sleep(15)
    pool = ocean.pool.create(
        data.token_address,
        data_token_amount=data.tokens_nomber,
        OCEAN_amount=data.ocean_amount,
        from_wallet=wallet
    )
    rospy.loginfo(f'Pool created with address {pool.address}')
    pool_address = pool.address
    resp = TokenResponse()
    resp.token_address = data.token_address
    resp.did = data.did
    resp.pool_address = pool_address
    #rospy.loginfo(f'Publishing result {resp}')
    pub_response.publish(resp)


rospy.Subscriber('/ocean/create_pool', CreatingPool, create_pool_callback)
rospy.spin()
