import json
import boto3
import time

iot_data = boto3.client('iot-data', region_name='us-east-2')

def lambda_handler(event, context):

    time.sleep(5) 

    try:
        print(f"📥 Evento recibido: {json.dumps(event)}")
        
        thing_name = event.get('thing_name', 'Object_PP')
        motion_detected = event.get('motion_detected', False)
        current_status = event.get('current_status', 'unknown')
        
        print(f"🔍 Thing: {thing_name}")
        print(f"🔍 Motion: {motion_detected}")
        print(f"🔍 Status actual: {current_status}")
        
        if not motion_detected:
            print("⏭️ Sin movimiento detectado, ignorando")
            return {
                'statusCode': 200,
                'body': json.dumps('Sin movimiento - no action')
            }
        
        if current_status not in ['closed', 'mid']:
            print(f"⏭️ Status '{current_status}' no requiere apertura")
            return {
                'statusCode': 200,
                'body': json.dumps('Ya está abierto o status inválido')
            }
        
        shadow_update = {
            "state": {
                "desired": {
                    "command": "open",
                    "servo1_angle": 90, 
                    "servo2_angle": 0
                }
            }
        }
        
        print(f"📤 Enviando comando 'open' al shadow...")
        
        response = iot_data.update_thing_shadow(
            thingName=thing_name,
            payload=json.dumps(shadow_update)
        )
        
        print(f"✅ Shadow actualizado exitosamente")
        print(f"📄 Respuesta: {response}")
        
        return {
            'statusCode': 200,
            'body': json.dumps({
                'message': 'Comando OPEN enviado correctamente',
                'thing_name': thing_name,
                'action': 'open'
            })
        }
        
    except Exception as e:
        print(f"❌ Error: {str(e)}")
        return {
            'statusCode': 500,
            'body': json.dumps(f'Error: {str(e)}')
        }