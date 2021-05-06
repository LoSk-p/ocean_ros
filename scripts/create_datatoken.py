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
import time
import os

rospy.init_node("datatoken_creator", anonymous=False)
config_path = rospy.get_param("~config")
print(config_path)
pub_response = rospy.Publisher("token_response", TokenResponse, queue_size=10)

config = Config(config_path)
ocean = Ocean(config)

def create_datatoken_callback(data):
    rospy.loginfo(f'Got create datatoken request')
    wallet = Wallet(ocean.web3, private_key=data.private_key)
    data_token = ocean.create_data_token('DataToken1', 'DT1', wallet, blob=ocean.config.metadata_cache_uri)
    token_address = data_token.address
    rospy.loginfo(f'Created datatoken with address {token_address}')
    files = []
    for i in range(len(data.files_url)):
        files.append({"index": i, "contentType": data.files_content_type[i],
                "url": data.files_url[i]})
    metadata =  {
        "main": {
            "type": data.type, "name": data.name, "author": data.author,
            "license": data.license, "dateCreated": data.data_created,
            "files": files}
        }
    service_attributes = {
            "main": {
                "name": "dataAssetAccessServiceAgreement",
                "creator": wallet.address,
                "timeout": 3600 * 24,
                "datePublished": data.data_created,
                "cost": 1.0, # <don't change, this is obsolete>
            }
        }
    service_endpoint = DataServiceProvider.get_url(ocean.config)
    download_service = ServiceDescriptor.access_service_descriptor(service_attributes, service_endpoint)
    asset = ocean.assets.create(
        metadata,
        wallet,
        service_descriptors=[download_service],
        data_token_address=token_address)
    did = asset.did
    rospy.loginfo(f'did: {did}')
    rospy.loginfo(f'Minting datatokens')
    data_token.mint_tokens(wallet.address, data.tokens_nomber, wallet)
    OCEAN_token = BToken(ocean.OCEAN_address)
    assert OCEAN_token.balanceOf(wallet.address) > 0, "need OCEAN"
    rospy.loginfo(f'Creating pool')
    time.sleep(15)
    pool = ocean.pool.create(
        token_address,
        data_token_amount=data.tokens_nomber,
        OCEAN_amount=data.ocean_amount,
        from_wallet=wallet
    )
    rospy.loginfo(f'Pool created with address {pool.address}')
    pool_address = pool.address
    resp = TokenResponse()
    resp.token_address = token_address
    resp.did = did
    resp.pool_address = pool_address
    #rospy.loginfo(f'Publishing result {resp}')
    pub_response.publish(resp)


rospy.Subscriber('/create_datatoken', Metadata, create_datatoken_callback)
rospy.spin()
