# -*- coding: utf-8 -*-
"""
Filename: plc_grpc_client.py
Author: Jianming.Liu
Date: 2025/10/28 
Description:
 
"""
import grpc
from grpc import StatusCode
from proto import PLC_GRPC_pb2
from proto import PLC_GRPC_pb2_grpc


class PlcClient:
    def __init__(self, host: str = 'localhost', port: int = 50051):
        self.channel = grpc.insecure_channel(f'{host}:{port}')
        self.stub = PLC_GRPC_pb2_grpc.PLC_ServiceStub(self.channel)

    def interferometer_set_zero(self):
        try:
            request = PLC_GRPC_pb2.PLC_CMD_EmptyRequest()
            response_clear = self.stub.Interferometer_ClearZero(request)
            response_homing = self.stub.Interferometer_Homing(request)
            response_calculate_lambda = self.stub.Interferometer_CalculateLamda(request)
            response_start_calculate = self.stub.Interferometer_Start_Calculate(request)
            return [response_clear, response_homing, response_calculate_lambda, response_start_calculate]
        except grpc.RpcError as e:
            # gRPC 特定异常
            print(f"gRPC 错误: {e}")
            print(f"错误代码: {e.code()}")  # 状态码
            print(f"错误详情: {e.details()}")  # 详细描述

            # 根据不同的状态码处理
            if e.code() == StatusCode.UNAVAILABLE:
                print("服务不可用，请检查服务端是否启动")
            elif e.code() == StatusCode.DEADLINE_EXCEEDED:
                print("请求超时")
            elif e.code() == StatusCode.NOT_FOUND:
                print("资源未找到")
            elif e.code() == StatusCode.PERMISSION_DENIED:
                print("权限不足")

            return None

    def camera_trigger(self, camera_id):
        try:
            request = PLC_GRPC_pb2.PLC_CMD_EmptyRequest()
            if camera_id == 0:
                response = self.stub.DownCameraXNTakePhoto(request)
            elif camera_id == 1:
                response = self.stub.DownPreProcess_XP_Start(request)
            elif camera_id == 2:
                response = self.stub.UpPreProcess_Start(request)
            elif camera_id == 3:
                response = self.stub.CoarseProcess_Start(request)
            return response

        except grpc.RpcError as e:
            # gRPC 特定异常
            print(f"gRPC 错误: {e}")
            print(f"错误代码: {e.code()}")  # 状态码
            print(f"错误详情: {e.details()}")  # 详细描述

            # 根据不同的状态码处理
            if e.code() == StatusCode.UNAVAILABLE:
                print("服务不可用，请检查服务端是否启动")
            elif e.code() == StatusCode.DEADLINE_EXCEEDED:
                print("请求超时")
            elif e.code() == StatusCode.NOT_FOUND:
                print("资源未找到")
            elif e.code() == StatusCode.PERMISSION_DENIED:
                print("权限不足")

            return None



def main():
    pass


if __name__ == '__main__':
    main()
