import json


def AO_Illumination_LPC_LightControlWord(task="CMD_SetAo", value=3):
    cmd = {
        "context": "",
        "result": False,
        "task": "CMD_SetAo"
    }
    payload = {"Ao": 200, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    with open(file_path, 'w') as payloadon_file:
        json.dump(cmd, payloadon_file, indent=1)

    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    # payloadon.dump()将数据转换成payloadon数据，然后写入文件中 payloadon.dumps()将python对象转换为payloadon格式的字符串，不写入文件。

    # json_str = payloadon.dumps(cmd, indent=1)
    # bytes = json_str.encode('utf-8', 'ignore')
    # print(bytes)
    ##{
    #   "context": "{\n    \"lpcLightControlWord\": 1,\n    \"lpcControlMode\": 0,\n    \"lpcTargetPower\": 0.0,\n    \"lpcTargetCurrent\": 0.0,\n    \"lpcSetLightPathMode\": 0,\n    \"lpcSetTargetDose\": 0,\n    \"lpcSetPeriodDoseIntegral\": 0,\n    \"lpcSetSamplingInterVal\": 0,\n    \"lpcSetControlWord\": 0\n}\n",
    #   "result": false,
    #   "task": "CMD_AoIllumLpcLightControlWord"
    # }
    return json_str


def AI_Illumination_ILC_ED1IntergalPower(task="CMD_AiValue"):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    # payload = {'0': {'Ai': 119, 'name': '', 'value': False}}
    payload = {"Ai": 121, "name": "", "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    with open(file_path, 'w') as payloadon_file:
        json.dump(cmd, payloadon_file, indent=1)

    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    # payloadon.dump()将数据转换成payloadon数据，然后写入文件中 payloadon.dumps()将python对象转换为payloadon格式的字符串，不写入文件。

    return json_str


def AI_Illumination_ILC_ESSIntergalPower(task="CMD_AiValue"):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    # payload = {'0': {'Ai': 119, 'name': '', 'value': False}}
    payload = {"Ai": 123, "name": "", "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    with open(file_path, 'w') as payloadon_file:
        json.dump(cmd, payloadon_file, indent=1)

    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    # payloadon.dump()将数据转换成payloadon数据，然后写入文件中 payloadon.dumps()将python对象转换为payloadon格式的字符串，不写入文件。

    return json_str


def AI_Illumination_ILC_OrignED1(task="CMD_AiValue"):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    # payload = {'0': {'Ai': 119, 'name': '', 'value': False}}
    payload = {"Ai": 124, "name": "", "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    with open(file_path, 'w') as payloadon_file:
        json.dump(cmd, payloadon_file, indent=1)

    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    # payloadon.dump()将数据转换成payloadon数据，然后写入文件中 payloadon.dumps()将python对象转换为payloadon格式的字符串，不写入文件。

    return json_str


def AI_Illumination_ILC_OrignED2(task="CMD_AiValue"):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    # payload = {'0': {'Ai': 119, 'name': '', 'value': False}}
    payload = {"Ai": 125, "name": "", "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    with open(file_path, 'w') as payloadon_file:
        json.dump(cmd, payloadon_file, indent=1)

    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AI_Illumination_ILC_OrignESS(task="CMD_AiValue"):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ai": 126, "name": "", "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    with open(file_path, 'w+') as payloadon_file:
        json.dump(cmd, payloadon_file, indent=1)

    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    # payloadon.dump()将数据转换成payloadon数据，然后写入文件中 payloadon.dumps()将python对象转换为payloadon格式的字符串，不写入文件。

    return json_str


def AI_Iiiumination_ILC_SamplingDone_127(task="CMD_AiValue", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ai": 127, "name": "", "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    with open(file_path, 'w+') as payloadon_file:
        json.dump(cmd, payloadon_file, indent=1)

    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    # payloadon.dump()将数据转换成payloadon数据，然后写入文件中 payloadon.dumps()将python对象转换为payloadon格式的字符串，不写入文件。

    return json_str


def AO_Iiiumination_ILC_SetControlWord_208(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 207, "name": "", "targetValue": value, "value": 0}
    cmd["context"] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    with open(file_path, 'w+') as payloadon_file:
        json.dump(cmd, payloadon_file, indent=1)

    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_ILCWorkMode_215(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 214, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_SetLightPathMode_204(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 204, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)

    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    with open(file_path, 'w+') as payloadon_file:
        json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_SetPeriodDoseIntegral_206(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 206, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_SetTargetDose_205(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 205, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_Shutter1DelayTime_216(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 215, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_Shutter2DelayTime_217(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 216, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_SetED1_k1_209(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 208, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_SetED1_b1_210(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 209, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_SetED2_k2_211(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 210, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_SetED2_b2_212(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 211, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_SetESS_k3_213(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 212, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Iiiumination_ILC_SetESS_b3_214(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 213, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Illumination_ILC_DisConVAGear(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 221, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Illumination_ILC_FilterGear(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 222, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Illumination_ILC_Shutter1Gear(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 223, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str


def AO_Illumination_LPC_SetDoseK(task="CMD_SetAo", value=0):
    cmd = {
        'context': "",
        'result': False,
        'task': task
    }
    payload = {"Ao": 229, "name": "", "targetValue": value, "value": 0}
    cmd['context'] = json.dumps(payload, indent=1)
    ############################################输出到json文件，用来查看对比原数据
    print(cmd)
    # file_path = 'GUI_IO_CMD_AiGroupStatus.json'
    # with open(file_path, 'w+') as payloadon_file:
    #     json.dump(cmd, payloadon_file, indent=1)
    json_str = json.dumps(cmd, indent=1)
    print(json_str)
    return json_str
