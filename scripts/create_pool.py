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


class PoolCreator():
    def __init__(self):
        rospy.init_node("pool_creator", anonymous=False)
        config_path = rospy.get_param("~config")

        self.pub_response = rospy.Publisher("token_response", TokenResponse, queue_size=10)

        config = Config(config_path)
        self.ocean = Ocean(config)
        rospy.Subscriber('/ocean/create_pool', CreatingPool, self.create_pool_callback)
        rospy.loginfo('pool_creator is ready')

    def create_pool_callback(self, data):
        rospy.loginfo(f'Got create pool request')
        with open(data.private_key_path, 'r') as f:
            private_key = f.read().rstrip()
        wallet = Wallet(self.ocean.web3, private_key=private_key)
        OCEAN_token = BToken(self.ocean.OCEAN_address)
        assert OCEAN_token.balanceOf(wallet.address) > 0, "need OCEAN"
        rospy.loginfo(f'Creating pool')
        time.sleep(15)
        try:
            pool = self.ocean.pool.create(
                data.token_address,
                data_token_amount=data.tokens_nomber,
                OCEAN_amount=data.ocean_amount,
                from_wallet=wallet
            )
        except Exception as e:
            rospy.loginfo(f'Got {e}')
            rospy.loginfo(f'Trying again')
            time.sleep(10)
            pool = self.ocean.pool.create(
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
        self.pub_response.publish(resp)
    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    PoolCreator().spin()
