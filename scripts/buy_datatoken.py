#!/usr/bin/env python3
import rospy
from ocean_lib.config import Config
from ocean_lib.ocean.ocean import Ocean
from ocean_lib.web3_internal.wallet import Wallet
from ocean_lib.data_provider.data_service_provider import DataServiceProvider
from ocean_lib.common.agreements.service_factory import ServiceDescriptor
from ocean_lib.common.agreements.service_types import ServiceTypes
from ocean_lib.models.btoken import BToken
from ocean_ros.msg import BuyingResponse
from ocean_ros.msg import BuyDatatoken
import time
import os

class DatatokenDownloader():
    def __init__(self):
        rospy.init_node("datatoken_downloader", anonymous=False)
        config_path = rospy.get_param("~config")
        self.buy_response = rospy.Publisher("buying_response", BuyingResponse, queue_size=10)
        rospy.Subscriber('/ocean/get_datatoken', BuyDatatoken, self.get_datatoken_callback)

        config = Config(config_path)
        self.bob_ocean = Ocean(config)
        rospy.loginfo(f'datatoken_downloader is ready')

    def buy_token(self, wallet, pool_address):
        self.bob_ocean.pool.buy_data_tokens(
            pool_address, 
            amount=1.0, # buy 1.0 datatoken
            max_OCEAN_amount=10.0, # pay up to 10.0 OCEAN
            from_wallet=wallet
        )

    def get_datatoken_callback(self, data):
        rospy.loginfo("Got buy datatoken request")
        with open(data.private_key_path, 'r') as f:
            private_key = f.read().rstrip()
        bob_wallet = Wallet(self.bob_ocean.web3, private_key=private_key)
        assert self.bob_ocean.web3.eth.getBalance(bob_wallet.address) > 0, "need Rinkeby ETH"
        OCEAN_token = BToken(self.bob_ocean.OCEAN_address)
        assert OCEAN_token.balanceOf(bob_wallet.address) > 0, "need Rinkeby OCEAN"
        data_token = self.bob_ocean.get_data_token(data.token_address)
        rospy.loginfo("Buying datatokens")
        time.sleep(10)
        try:
            self.buy_token(bob_wallet, data.pool_address)
            rospy.loginfo(f"You have {data_token.token_balance(bob_wallet.address)} datatokens.")
            assert data_token.balanceOf(bob_wallet.address) >= 1.0, "You didn't get 1.0 datatoken"
        except Exception as e:
            rospy.loginfo(f'Got {e}')
            rospy.loginfo(f'Trying again')
            time.sleep(10)
            self.buy_token(bob_wallet, data.pool_address)
            rospy.loginfo(f"You have {data_token.token_balance(bob_wallet.address)} datatokens.")
            assert data_token.balanceOf(bob_wallet.address) >= 1.0, "You didn't get 1.0 datatoken"
        
        fee_receiver = None
        asset = self.bob_ocean.assets.resolve(data.did)
        service = asset.get_service(ServiceTypes.ASSET_ACCESS)
        quote = self.bob_ocean.assets.order(asset.did, bob_wallet.address, service_index=service.index)
        time.sleep(10)
        order_tx_id = self.bob_ocean.assets.pay_for_service(quote.amount, quote.data_token_address, asset.did, service.index, fee_receiver, bob_wallet)
        rospy.loginfo(f"order_tx_id = '{order_tx_id}'")
        if data.destination[-1] != '/':
            dest = f"{data.destination}/"
        else:
            dest = data.destination
        file_path = self.bob_ocean.assets.download(
            asset.did, 
            service.index, 
            bob_wallet, 
            order_tx_id, 
            destination=dest
        )
        rospy.loginfo(f"file_path = '{file_path}'")
        self.buy_response.publish(file_path)

    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    DatatokenDownloader().spin()