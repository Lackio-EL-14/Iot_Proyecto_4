import json
import boto3
import time

iot_data = boto3.client('iot-data', region_name='us-east-2')

def lambda_handler(event, context):
    thing_name = event.get('thing_name', 'Object_PP')
    
    print("⏳ Iniciando espera de 5 segundos...")
    time.sleep(5) 
    
    try:
        response = iot_data.get_thing_shadow(thingName=thing_name)
        payload = json.loads(response['payload'].read())

        actual_motion = payload['state']['reported'].get('motion_detected', False)
        
        if actual_motion:
            print("🚫 CANCELADO: Se detectó movimiento durante la espera.")
            return 
            
    except Exception as e:
        print(f"Error leyendo shadow: {str(e)}")


    shadow_update = {
        "state": {
            "desired": {
                "command": "close",
                "servo1_angle": 0, 
                "servo2_angle": 90

            }
        }
    }
    
    iot_data.update_thing_shadow(
        thingName=thing_name,
        payload=json.dumps(shadow_update)
    )
    
    return {'statusCode': 200, 'body': 'Cerrado tras espera'}