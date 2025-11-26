import logging
import ask_sdk_core.utils as ask_utils
import boto3
import json

from ask_sdk_core.skill_builder import SkillBuilder
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.dispatch_components import AbstractExceptionHandler
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


DYNAMODB_TABLE = 'user_thing'
AWS_REGION = 'us-east-2'


dynamodb = boto3.resource('dynamodb', region_name=AWS_REGION)
iot_client = boto3.client('iot-data', region_name=AWS_REGION)


def get_thing_name(user_id):
    """Obtiene el nombre del Thing asociado al usuario desde DynamoDB"""
    try:
        table = dynamodb.Table(DYNAMODB_TABLE)
        response = table.get_item(Key={'user_id': user_id})
        
        if 'Item' in response:
            thing_name = response['Item']['thing_name']
            logger.info(f" Thing: {thing_name} | User: {user_id}")
            return thing_name
        else:
            logger.error(f" No thing found for user: {user_id}")
            return None
            
    except Exception as e:
        logger.error(f" DynamoDB error: {str(e)}")
        return None


def get_thing_shadow(thing_name):
    """Obtiene el Shadow del Thing desde AWS IoT"""
    try:
        response = iot_client.get_thing_shadow(thingName=thing_name)
        streaming_body = response['payload']
        json_state = json.loads(streaming_body.read())
        
        logger.info(f" Shadow obtenido: {thing_name}")
        return json_state
        
    except Exception as e:
        logger.error(f" Error obteniendo shadow: {str(e)}")
        return None


def update_thing_shadow(thing_name, desired_state):
    """
    Actualiza el estado deseado del Shadow
    El ESP32 recibirá esto como DELTA y lo procesará
    """
    try:
        payload = {
            "state": {
                "desired": desired_state
            }
        }
        
        logger.info(f" Actualizando Shadow: {json.dumps(desired_state)}")
        
        response = iot_client.update_thing_shadow(
            thingName=thing_name,
            payload=json.dumps(payload)
        )
        
        logger.info(f" Shadow actualizado correctamente")
        return True
        
    except Exception as e:
        logger.error(f" Error actualizando shadow: {str(e)}")
        return False


# ============================================
# HANDLERS DE ALEXA
# ============================================

class LaunchRequestHandler(AbstractRequestHandler):
    """Handler cuando se inicia la skill"""
    def can_handle(self, handler_input):
        return ask_utils.is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        speak_output = ("Bienvenido al basurero inteligente. "
                       "Puedes decir: abre completamente, cierra totalmente, "
                       "posición media, estado, hay movimiento, o infrarrojo. "
                       "¿Qué deseas hacer?")

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask("¿Qué deseas hacer?")
                .response
        )


class OpenServoIntentHandler(AbstractRequestHandler):
    """
    Handler: OpenServoIntent
    Invocations: "apertura total", "abre ambos servos", "abre completamente"
    Abre AMBOS servos sincronizadamente
    """
    def can_handle(self, handler_input):
        return ask_utils.is_intent_name("OpenServoIntent")(handler_input)

    def handle(self, handler_input):
        user_id = handler_input.request_envelope.session.user.user_id
        logger.info(f"🎤 OpenServoIntent | User: {user_id}")
        
        thing_name = get_thing_name(user_id)
        
        if not thing_name:
            speak_output = "No pude encontrar tu dispositivo. Verifica la configuración."
            return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response
        
        # Actualizar Shadow para abrir AMBOS servos
        success = update_thing_shadow(thing_name, {
            "servo1_angle": 180,
            "servo2_angle": 180,
            "command": "open"
        })
        
        speak_output = "Abriendo la tapa del basurero." if success else "Hubo un error al enviar el comando."
        
        return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response


class CloseServoIntentHandler(AbstractRequestHandler):
    """
    Handler: CloseServoIntent
    Invocations: "pon los servos en cero", "cierra totalmente", "cierra completamente"
    Cierra AMBOS servos sincronizadamente
    """
    def can_handle(self, handler_input):
        return ask_utils.is_intent_name("CloseServoIntent")(handler_input)

    def handle(self, handler_input):
        user_id = handler_input.request_envelope.session.user.user_id
        logger.info(f"🎤 CloseServoIntent | User: {user_id}")
        
        thing_name = get_thing_name(user_id)
        
        if not thing_name:
            speak_output = "No pude encontrar tu dispositivo."
            return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response
        
        
        success = update_thing_shadow(thing_name, {
            "servo1_angle": 0,
            "servo2_angle": 0,
            "command": "close"
        })
        
        speak_output = "Cerrando la tapa del basurero." if success else "Error al cerrar."
        
        return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response


class MidPositionIntentHandler(AbstractRequestHandler):
    """
    Handler: MidPositionIntent
    Invocations: "ambos al medio", "noventa grados", "posición media"
    Posiciona AMBOS servos a 90 grados
    """
    def can_handle(self, handler_input):
        return ask_utils.is_intent_name("MidPositionIntent")(handler_input)

    def handle(self, handler_input):
        user_id = handler_input.request_envelope.session.user.user_id
        logger.info(f"🎤 MidPositionIntent | User: {user_id}")
        
        thing_name = get_thing_name(user_id)
        
        if not thing_name:
            speak_output = "No pude encontrar tu dispositivo."
            return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response
        
        
        success = update_thing_shadow(thing_name, {
            "servo1_angle": 90,
            "servo2_angle": 90,
            "command": "mid"
        })
        
        speak_output = "Posición media: 90 grados." if success else "Error al mover."
        
        return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response


class GetStatusIntentHandler(AbstractRequestHandler):
    """
    Handler: GetStatusIntent
    Invocations: "como esta todo", "cual es el estado", "estado", "reporte de estado"
    Reporta estado de AMBOS servos y AMBOS sensores
    """
    def can_handle(self, handler_input):
        return ask_utils.is_intent_name("GetStatusIntent")(handler_input)

    def handle(self, handler_input):
        user_id = handler_input.request_envelope.session.user.user_id
        logger.info(f"🎤 GetStatusIntent | User: {user_id}")
        
        thing_name = get_thing_name(user_id)
        
        if not thing_name:
            speak_output = "No pude encontrar tu dispositivo."
            return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response
        
        shadow = get_thing_shadow(thing_name)
        
        if not shadow or 'state' not in shadow:
            speak_output = "No pude obtener el estado del dispositivo."
            return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response
        
        reported = shadow['state'].get('reported', {})
        
       
        servo1_angle = reported.get('servo1_angle', 0)
        servo2_angle = reported.get('servo2_angle', 0)
        status = reported.get('status', 'desconocido')
        
     
        motion_detected = reported.get('motion_detected', False)
        infrared_detected = reported.get('infrared_detected', False)
        
       
        servo_status = f"La tapa está en {servo1_angle} grados"
        if status == "open":
            servo_status = "La tapa está completamente abierta"
        elif status == "closed":
            servo_status = "La tapa está completamente cerrada"
        
        motion_status = "hay movimiento detectado" if motion_detected else "no hay movimiento"
        ir_status = "hay un objeto cerca" if infrared_detected else "no hay objetos cerca"
        
        speak_output = f"{servo_status}. {motion_status}, y {ir_status}."
        
        logger.info(f"📊 Estado: Servo1={servo1_angle}° Servo2={servo2_angle}° | Motion={motion_detected} | IR={infrared_detected}")
        
        return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response


class CheckMotionIntentHandler(AbstractRequestHandler):
    """
    Handler: CheckMotionIntent
    Invocations: "sensor de movimiento", "se detecta algun movimiento", "hay movimiento"
    Consulta el sensor PIR
    """
    def can_handle(self, handler_input):
        return ask_utils.is_intent_name("CheckMotionIntent")(handler_input)

    def handle(self, handler_input):
        user_id = handler_input.request_envelope.session.user.user_id
        logger.info(f"🎤 CheckMotionIntent | User: {user_id}")
        
        thing_name = get_thing_name(user_id)
        
        if not thing_name:
            speak_output = "No pude encontrar tu dispositivo."
            return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response
        
        shadow = get_thing_shadow(thing_name)
        
        if not shadow:
            speak_output = "No pude obtener el estado del sensor de movimiento."
            return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response
        
        reported = shadow['state'].get('reported', {})
        motion_detected = reported.get('motion_detected', False)
        
        speak_output = ("Sí, el sensor de movimiento detectó actividad." if motion_detected 
                       else "No, el sensor de movimiento no ha detectado nada.")
        
        logger.info(f"🚨 Motion: {motion_detected}")
        
        return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response


class CheckIRIntentHandler(AbstractRequestHandler):
    """
    Handler: CheckIRIntent
    Invocations: "infrarrojo", "obstaculo", "hay algun obstaculo"
    Consulta el sensor infrarrojo
    """
    def can_handle(self, handler_input):
        return ask_utils.is_intent_name("CheckIRIntent")(handler_input)

    def handle(self, handler_input):
        user_id = handler_input.request_envelope.session.user.user_id
        logger.info(f"🎤 CheckIRIntent | User: {user_id}")
        
        thing_name = get_thing_name(user_id)
        
        if not thing_name:
            speak_output = "No pude encontrar tu dispositivo."
            return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response
        
        shadow = get_thing_shadow(thing_name)
        
        if not shadow:
            speak_output = "No pude obtener el estado del sensor infrarrojo."
            return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response
        
        reported = shadow['state'].get('reported', {})
        infrared_detected = reported.get('infrared_detected', False)
        
        speak_output = ("Sí, el sensor infrarrojo detectó un objeto cerca." if infrared_detected 
                       else "No, el sensor infrarrojo no detecta ningún objeto.")
        
        logger.info(f"📡 Infrared: {infrared_detected}")
        
        return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response


# ============================================
# HANDLERS ESTÁNDAR DE ALEXA
# ============================================

class HelpIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return ask_utils.is_intent_name("AMAZON.HelpIntent")(handler_input)

    def handle(self, handler_input):
        speak_output = ("Puedo controlar tu basurero inteligente. "
                       "Puedes decir: abre completamente, cierra totalmente, "
                       "posición media, estado, hay movimiento, o infrarrojo. "
                       "¿Qué deseas hacer?")

        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask(speak_output)
                .response
        )


class CancelOrStopIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return (ask_utils.is_intent_name("AMAZON.CancelIntent")(handler_input) or
                ask_utils.is_intent_name("AMAZON.StopIntent")(handler_input))

    def handle(self, handler_input):
        speak_output = "Hasta luego!"
        return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response


class FallbackIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return ask_utils.is_intent_name("AMAZON.FallbackIntent")(handler_input)

    def handle(self, handler_input):
        speech = ("No entendí eso. Puedes decir: "
                 "abre completamente, cierra totalmente, estado, "
                 "hay movimiento, o infrarrojo. "
                 "¿Qué deseas hacer?")
        reprompt = "¿En qué puedo ayudarte?"
        
        return handler_input.response_builder.speak(speech).ask(reprompt).ask("¿Algo más?").response


class SessionEndedRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        return ask_utils.is_request_type("SessionEndedRequest")(handler_input)

    def handle(self, handler_input):
        logger.info("Session ended")
        return handler_input.response_builder.response


class IntentReflectorHandler(AbstractRequestHandler):
    """Handler de reflejo - solo para debugging"""
    def can_handle(self, handler_input):
        return ask_utils.is_request_type("IntentRequest")(handler_input)

    def handle(self, handler_input):
        intent_name = ask_utils.get_intent_name(handler_input)
        speak_output = f"Activaste {intent_name}."
        
        return handler_input.response_builder.speak(speak_output).ask("¿Algo más?").response


class CatchAllExceptionHandler(AbstractExceptionHandler):
    """Handler genérico de errores"""
    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):
        logger.error(exception, exc_info=True)
        
        speak_output = "Perdón, tuve un problema procesando tu solicitud. Intenta de nuevo."
        
        return (
            handler_input.response_builder
                .speak(speak_output)
                .ask("¿Quieres intentar de nuevo?")
                .response
        )



sb = SkillBuilder()

# Agregar handlers personalizados (ORDEN IMPORTA)
sb.add_request_handler(LaunchRequestHandler())
sb.add_request_handler(OpenServoIntentHandler())
sb.add_request_handler(CloseServoIntentHandler())
sb.add_request_handler(MidPositionIntentHandler())
sb.add_request_handler(GetStatusIntentHandler())
sb.add_request_handler(CheckMotionIntentHandler())
sb.add_request_handler(CheckIRIntentHandler())

# Agregar handlers estándar
sb.add_request_handler(HelpIntentHandler())
sb.add_request_handler(CancelOrStopIntentHandler())
sb.add_request_handler(FallbackIntentHandler())
sb.add_request_handler(SessionEndedRequestHandler())
sb.add_request_handler(IntentReflectorHandler())  # Debe ir al final

sb.add_exception_handler(CatchAllExceptionHandler())

lambda_handler = sb.lambda_handler()