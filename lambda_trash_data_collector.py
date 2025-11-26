import json
import boto3
from decimal import Decimal

dynamodb = boto3.resource('dynamodb')
table = dynamodb.Table('trash_data')

def lambda_handler(event, context):
    try:
        # Validar que haya datos relevantes
        motion = event.get('motion_detected', False)
        infrared = event.get('infrared_detected', False)
        command = event.get('command', 'idle')
        
        # Log de entrada para debug
        print(f"Evento recibido: motion={motion}, infrared={infrared}, command={command}")
        
        # Filtro adicional en Lambda (doble verificación)
        if not motion and not infrared and command == 'idle':
            print("⏭️ Evento ignorado: sin detección relevante")
            return {
                "statusCode": 200,
                "body": json.dumps("Evento ignorado (sin detección)")
            }
        
        # Preparar item optimizado para DynamoDB
        data_item = {
            'thing_name': event.get('thing_name', 'unknown'),
            'timestamp': Decimal(str(event.get('timestamp', 0))),
            'motion_detected': motion,
            'infrared_detected': infrared,
            'status': event.get('status', 'unknown')
        }
        
        # Guardar en DynamoDB
        response = table.put_item(Item=data_item)
        
        print(f" Datos guardados: {json.dumps(data_item, default=str)}")
        
        return {
            "statusCode": 200,
            "body": json.dumps("Datos guardados correctamente")
        }
        
    except Exception as e:
        print(f" Error guardando datos: {str(e)}")
        return {
            "statusCode": 500,
            "body": json.dumps(f"Error: {str(e)}")
        }