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

rospy.init_node("datatoken_downloader", anonymous=False)
config_path = rospy.get_param("~config")
print(config_path)
buy_response = rospy.Publisher("buying_response", BuyingResponse, queue_size=10)

config = Config(config_path)
bob_ocean = Ocean(config)

def get_datatoken_callback(data):
    rospy.loginfo("Got buy datatoken request")
    bob_wallet = Wallet(bob_ocean.web3, private_key=data.private_key)
    assert bob_ocean.web3.eth.getBalance(bob_wallet.address) > 0, "need Rinkeby ETH"
    OCEAN_token = BToken(bob_ocean.OCEAN_address)
    assert OCEAN_token.balanceOf(bob_wallet.address) > 0, "need Rinkeby OCEAN"
    data_token = bob_ocean.get_data_token(data.token_address)
    rospy.loginfo("Buying datatokens")
    time.sleep(10)
    bob_ocean.pool.buy_data_tokens(
        data.pool_address, 
        amount=1.0, # buy 1.0 datatoken
        max_OCEAN_amount=10.0, # pay up to 10.0 OCEAN
        from_wallet=bob_wallet
    )
    rospy.loginfo(f"You have {data_token.token_balance(bob_wallet.address)} datatokens.")
    assert data_token.balanceOf(bob_wallet.address) >= 1.0, "You didn't get 1.0 datatoken"
    fee_receiver = None
    asset = bob_ocean.assets.resolve(data.did)
    service = asset.get_service(ServiceTypes.ASSET_ACCESS)
    quote = bob_ocean.assets.order(asset.did, bob_wallet.address, service_index=service.index)
    time.sleep(10)
    order_tx_id = bob_ocean.assets.pay_for_service(quote.amount, quote.data_token_address, asset.did, service.index, fee_receiver, bob_wallet)
    rospy.loginfo(f"order_tx_id = '{order_tx_id}'")
    if data.destination[-1] != '/':
        dest = f"{data.destination}/"
    else:
        dest = data.destination
    file_path = bob_ocean.assets.download(
        asset.did, 
        service.index, 
        bob_wallet, 
        order_tx_id, 
        destination=dest
    )
    rospy.loginfo(f"file_path = '{file_path}'")
    buy_response.publish(file_path)



rospy.Subscriber('/get_datatoken', BuyDatatoken, get_datatoken_callback)
rospy.spin()