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


class DatatokenCreator():
    def __init__(self):
        rospy.init_node("datatoken_creator", anonymous=False)
        config_path = rospy.get_param("~config")
        self.create_pool = rospy.Publisher("create_pool", CreatingPool, queue_size=10)
        rospy.Subscriber('/ocean/create_datatoken', Metadata, self.create_datatoken_callback)
        config = Config(config_path)
        self.ocean = Ocean(config)
        rospy.loginfo(f'datatoken_creator is ready')

    def create_datatoken_callback(self, data):
        rospy.loginfo(f'Got create datatoken request')
        with open(data.private_key_path, 'r') as f:
            private_key = f.read().rstrip()
        wallet = Wallet(self.ocean.web3, private_key=private_key)
        try:
            data_token = self.ocean.create_data_token('DataToken1', 'DT1', wallet, blob=self.ocean.config.metadata_cache_uri)
        except Exception as e:
            rospy.loginfo(f'Got {e}')
            rospy.loginfo(f'Trying again')
            time.sleep(10)
            data_token = self.ocean.create_data_token('DataToken1', 'DT1', wallet, blob=self.ocean.config.metadata_cache_uri)
        time.sleep(5)
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
        print(metadata)
        service_attributes = {
                "main": {
                    "name": "dataAssetAccessServiceAgreement",
                    "creator": wallet.address,
                    "timeout": 3600 * 24,
                    "datePublished": data.data_created,
                    "cost": 1.0, # <don't change, this is obsolete>
                }
            }
        time.sleep(5)
        service_endpoint = DataServiceProvider.get_url(self.ocean.config)
        download_service = ServiceDescriptor.access_service_descriptor(service_attributes, service_endpoint)
        time.sleep(5)
        try:
            asset = self.ocean.assets.create(
                metadata,
                wallet,
                service_descriptors=[download_service],
                data_token_address=token_address)
        except Exception as e:
            rospy.loginfo(f'Got {e}')
            rospy.loginfo(f'Trying again')
            time.sleep(10)
            asset = self.ocean.assets.create(
                metadata,
                wallet,
                service_descriptors=[download_service],
                data_token_address=token_address)
        did = asset.did
        rospy.loginfo(f'did: {did}')
        rospy.loginfo(f'Minting datatokens')
        time.sleep(15)
        data_token.mint_tokens(wallet.address, float(data.tokens_nomber), wallet)
        rospy.loginfo(f"Minted {data.tokens_nomber} datatokens")
        time.sleep(15)

        msg = CreatingPool()
        msg.private_key_path = data.private_key_path
        msg.token_address = token_address
        msg.did = did
        msg.tokens_nomber = data.tokens_nomber
        msg.ocean_amount = data.ocean_amount
        self.create_pool.publish(msg)
    
    def spin(self):
        rospy.spin()


if __name__ == '__main__':
    DatatokenCreator().spin()
