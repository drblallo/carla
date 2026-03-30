"""
Example module demonstrating MQTT publishing functionality with the STEP library.

This module showcases different approaches to publishing Cooperative Awareness Messages (CAM)
using the STEP library's MQTT communication features. It includes:

Key Features:

    - MQTT connection management with and without context managers
    - Publishing CAM messages in multiple formats (JSON, ORM, Dictionary)
    - Subscription handling for V2X messages
    - UTC-based logging configuration
    - Secure TLS communication setup

The module provides example implementations for:

    - Creating and configuring MQTT clients
    - Setting up secure connections with TLS
    - Message format conversions
    - Proper connection handling and cleanup
    - Logging with UTC timestamps
"""
import uuid
import time
import socket
# Standard Library Imports
from logging import Logger, getLogger, FileHandler, Formatter

import argparse
from time import gmtime as time_gmtime
import logging
from ssl import PROTOCOL_TLSv1_2
from ssl import VerifyMode as ssl_VerifyMode
from time import sleep as time_sleep
from typing import Any, Optional

# Third Party Imports
from aiomqtt import TLSParameters
from step_cloud_schemas import (
    ActionId,
    Altitude,
    CamDataV1,
    CauseCode,
    DenmDataV1,
    DriveDirection,
    Heading,
    Identification,
    ItsTime,
    LocationContainer,
    ManagementContainerDenm,
    MessageId,
    ObjectDimension,
    ObjectFace,
    PerceivedObject,
    PositionConfidenceEllipse,
    PreCrashContainer,
    ReferencePosition,
    RoadType,
    SituationContainer,
    Speed,
    Termination,
    TrafficDirection,
    TrafficParticipantType,
    VehicleLength,
    VehicleLengthConfidenceIndication,
    VehicleRole,
)

# step_lib Package Imports
from step_lib import (
    CamMessageV1,
    DenmMessageV1,
    GeoNetworkAddress,
    GpsLocation,
    MqttQoS,
    PublishTopicParams,
    SyncComManager,
    SyncComManagerParams,
)

from typing import List, Dict, Any, Optional


def sync_com_manager_create(
    logger: Logger, station_id: Optional[int] 
) -> SyncComManager:
    """
    Creates and configures a SyncComManager instance for MQTT communication.

    This function initializes a SyncComManager with specific TLS and communication parameters
    for secure MQTT messaging, particularly configured for V2X (Vehicle-to-Everything)
    communication using CAM and DENM messages.

    Args:
        logger (Logger): Logger instance for tracking operations and errors
        station_id (Optional[int]): Optional station identifier for the MQTT client, can be None

    Returns:
        SyncComManager: Configured instance of SyncComManager ready for MQTT communication

    Example:
        logger = Logger()
        station_id = 12345
        com_manager = sync_com_manager_create(logger, station_id)

    Note:
        The function uses predefined MQTT broker credentials and TLS parameters.
        It's configured to publish CAM and DENM PDUs on specific topics with geohashing.
    """
    hostname = socket.gethostname()[:6]
    unique_suffix = uuid.uuid4().hex[:8]
    instance_name = f"carla-{hostname}-{unique_suffix}-{station_id}"
    tls_params = TLSParameters(
        ca_certs=None,
        certfile=None,
        keyfile=None,
        cert_reqs=ssl_VerifyMode.CERT_REQUIRED,
        tls_version=PROTOCOL_TLSv1_2,
        ciphers=None,
        keyfile_password=None,
    )
    params = SyncComManagerParams(
        instance_name=instance_name, #"step_lib_carla",
        host="de-he-mn.mqtt.step.vodafone.com",
        port=8883,
        username="73f4f844-15c9-427b-8787-592b3c675d1d",
        password="a8d493c9-4214-4542-b8d7-ca1a137bfd73",
        station_id=None, 
        timeout=10,
        keepalive=30,
        bind_address="",
        bind_port=0,
        clean_start=True,
        queue_type=None,
        max_queued_incoming_messages=60,
        max_queued_outgoing_messages=60,
        max_inflight_messages=60,
        max_concurrent_outgoing_calls=60,
        tls_context=None,
        tls_params=tls_params,
        #tls_insecure=False,
        proxy=None,
        socket_options=None,
        session_expiry=0,
        receive_maximum=10,
        maximum_packet_size=4096,
        topic_alias_maximum=0,
        publish_topics_params=[
            PublishTopicParams(
                message_type=MessageId.CAM_PDU,
                header_topic="v2x/cam/264421_4",
                geohash_precision=8,
                qos=MqttQoS.QOS_0, # 0 non aspetto acknowledge, 1 lo aspetto
            ),
            # Non mandiamo DENM
            PublishTopicParams(
                message_type=MessageId.DENM_PDU,
                header_topic="v2x/denm/264421_4",
                geohash_precision=8,
                qos=MqttQoS.QOS_0,
            ),
        ],
        cache_dir="./.cache",
        logger=logger,
        reconnect_interval=2,
        max_incoming_queue_rooms=60,
        max_outgoing_queue_rooms=60,
    )
    return SyncComManager(params=params)


def create_cam_json_message() -> str:
    """
    Creates a JSON-formatted string containing vehicle information.
    This function returns a static JSON string that includes various vehicle and
    position parameters:

        - Creation time of the message
        - Station information (ID and type)
        - Reference position (latitude, longitude, and confidence metrics)
        - Altitude information
        - Vehicle heading
        - Speed data
        - Vehicle physical characteristics (length, width)
        - Motion parameters (acceleration, curvature, yaw rate)
        - Vehicle role information

    Returns:
        str: A JSON-formatted string containing standardized vehicle telemetry data
    """
    return """
    {
        "creationTime": "2024-12-20T17:31:38.725Z",
        "stationInfo": {"stationId": 2802522858, "stationType": "passengerCar"},
        "position": {
            "latitude": 25.4344459,
            "longitude": 51.232539899999999,
            "positionConfidence": {
                "semiMajorAxisLengthConfidence": 203,
                "semiMinorAxisLengthConfidence": 203,
                "semiMajorAxisOrientation": 0
            },
            "altitude": {"value": 3401, "confidence": 2}
        },
        "heading": {"value": 0, "confidence": null},
        "speed": {"value": 0, "confidence": null},
        "driveDirection": "forward",
        "vehicleLength": {
            "vehicleLengthValue": 30,
            "vehicleLengthConfidenceIndication": "noTrailerPresent"
        },
        "vehicleWidth": 10,
        "longitudinalAcceleration": null,
        "curvature": null,
        "curvatureCalculationMode": null,
        "yawRate": null,
        "vehicleRole": "default"
    }
    """


def create_cam_dict_message() -> dict[Any, Any]:
    """
    Creates a dictionary containing vehicle station information and positioning data.
    The dictionary includes detailed information about a vehicle's:

    - Station identification and type
    - Geographic position (latitude, longitude) with confidence metrics
    - Altitude
    - Movement parameters (heading, speed, direction)
    - Physical characteristics (length, width)
    - Vehicle role and status

    Returns:
        dict: A dictionary containing the following main keys:

            - creationTime: Timestamp of message creation
            - stationInfo: Station ID and type details
            - refPosition: Geographic coordinates and confidence metrics
            - refAltitude: Altitude value and confidence
            - heading: Directional heading and confidence
            - speed: Vehicle speed and confidence
            - driveDirection: Vehicle's drive direction
            - vehicleLength: Length value and trailer presence indication
            - vehicleWidth: Vehicle width in specified units
            - longitudinalAcceleration: Vehicle's forward/backward acceleration
            - curvature: Vehicle's turning radius
            - curvatureCalculationMode: Method used for curvature calculation
            - yawRate: Vehicle's rotation rate
            - vehicleRole: Designated role of the vehicle
    """
    return {
        "creationTime": str(object=ItsTime()),
        "stationInfo": {
            "stationId": 2802522858,
            "stationType": TrafficParticipantType.PASSENGER_CAR,
        },
        "position": {
            "latitude": 25.4344459,
            "longitude": 51.232539899999999,
            "positionConfidence": {
                "semiMajorAxisLengthConfidence": 203,
                "semiMinorAxisLengthConfidence": 203,
                "semiMajorAxisOrientation": 0,
            },
            "altitude": {"value": 3401, "confidence": 2},
        },
        "heading": {"value": 0, "confidence": None},
        "speed": {"value": 0, "confidence": None},
        "driveDirection": DriveDirection.FORWARD,
        "vehicleLength": {
            "vehicleLengthValue": 30,
            "vehicleLengthConfidenceIndication": VehicleLengthConfidenceIndication.NO_TRAILER_PRESENT,  # pylint: disable=line-too-long
        },
        "vehicleWidth": 10,
        "longitudinalAcceleration": None,
        "curvature": None,
        "curvatureCalculationMode": None,
        "yawRate": None,
        "vehicleRole": VehicleRole.DEFAULT,
    }


def create_cam_orm_message(current_position: GpsLocation, station_id: int = 1111111111, station_type: TrafficParticipantType = TrafficParticipantType.PASSENGER_CAR) -> CamMessageV1:
    """
    Creates and returns a CAM (Cooperative Awareness Message) V1 object with predefined values.

    The message contains information about a vehicle's:

        - Station identification (ID and type)
        - Geographic position (latitude, longitude, altitude)
        - Position confidence metrics
        - Basic motion state (heading, speed, direction)
        - Physical characteristics (length, width)
        - Role classification

    Returns:
        CamMessageV1: A structured CAM message object containing vehicle state and attributes
    """
    return CamMessageV1(
        message=CamDataV1(
            creationTime=str(object=ItsTime()),
            stationInfo=Identification(
                stationId=station_id,
                stationType=station_type, # Dice se auto, bus,truck (valorizzare meglio)
            ),
            position=ReferencePosition(
                latitude=current_position.latitude,
                longitude=current_position.longitude,
                positionConfidence=PositionConfidenceEllipse(
                    semiMajorAxisLengthConfidence=203,
                    semiMinorAxisLengthConfidence=203,
                    semiMajorAxisOrientation=0,
                ),
                altitude=Altitude(value=0, confidence=None),
            ),
            heading=Heading(
                value= current_position.heading,
                confidence=None,
            ),
            speed=Speed(value=current_position.speed, confidence=None),
            driveDirection=DriveDirection.FORWARD,
            vehicleLength=VehicleLength(
                vehicleLengthValue=30,
                vehicleLengthConfidenceIndication=VehicleLengthConfidenceIndication.NO_TRAILER_PRESENT,  # pylint: disable=line-too-long
            ),
            vehicleWidth=10,
            longitudinalAcceleration=None,
            curvature=None,
            curvatureCalculationMode=None,
            yawRate=None,
            vehicleRole=VehicleRole.DEFAULT,
        ),
    )

def create_denm_orm_message_static() -> DenmMessageV1:  # rinominata: usare create_denm_message()
    """
    Creates a DENMv1 (Decentralized Environmental Notification Message) ORM message.

    This function constructs a standardized DENM message used in V2X (Vehicle-to-Everything)
    communications containing various containers with detailed information about traffic events,
    vehicle status, and potential hazards.

    Returns:
        DenmMessageV1: A structured DENM message object containing:

            - Management Container: Basic event management information including position, timing,
              and station details
            - Situation Container: Information about the type of event/hazard
            - Location Container: Detailed positioning and road information
            - PreCrash Container: Collision prediction and object perception data

    Example message includes:

        - Station ID: 971391625
        - Event Position: Lat 41.9792448, Long 12.4923096
        - Event Type: Slow Vehicle
        - Speed: 450 (value) with 50 confidence
        - Time to Collision: 1256
    """
    return DenmMessageV1(
        message=DenmDataV1(
            stationId=971391625,
            managementContainer=ManagementContainerDenm(
                actionId=ActionId(originatingStationId=971391625, sequenceNumber=1),
                detectionTime="2024-12-20T17:31:38.725Z",
                referenceTime="2024-12-20T17:31:38.729Z",
                termination=Termination.CANCELLATION,
                eventPosition=ReferencePosition(
                    latitude=41.9792448,
                    longitude=12.4923096,
                    positionConfidence=PositionConfidenceEllipse(
                        semiMajorAxisLengthConfidence=500,
                        semiMinorAxisLengthConfidence=500,
                        semiMajorAxisOrientation=0,
                    ),
                    altitude=Altitude(value=100, confidence=2),
                ),
                awarenessDistance=199,
                trafficDirection=TrafficDirection.ALL_TRAFFIC_DIR,
                validityDuration=60,
                transmissionInterval=1000,
                stationType=TrafficParticipantType.PASSENGER_CAR,
            ),
            situationContainer=SituationContainer(
                informationQuality=None, eventType=CauseCode.SLOW_VEHICLE
            ),
            locationContainer=LocationContainer(
                eventSpeed=Speed(value=450, confidence=50),
                eventPositionHeading=Heading(value=0, confidence=50),
                roadType=RoadType.URBAN_NO_STRUCT_SEP_TO_OPPOSITE_LANES,
            ),
            preCrashContainer=PreCrashContainer(
                perceivedObject=PerceivedObject(
                    objectId=None,
                    measurementDeltaTime=2,
                    position=ReferencePosition(
                        latitude=41.9792102,
                        longitude=12.4923458,
                        positionConfidence=PositionConfidenceEllipse(
                            semiMajorAxisLengthConfidence=500,
                            semiMinorAxisLengthConfidence=500,
                            semiMajorAxisOrientation=0,
                        ),
                        altitude=Altitude(value=50, confidence=2),
                    ),
                    speed=Speed(value=23, confidence=2),
                    heading=Heading(value=125, confidence=2),
                    objectDimensionZ=ObjectDimension(value=100, confidence=2),
                    objectDimensionY=ObjectDimension(value=10, confidence=2),
                    objectDimensionX=ObjectDimension(value=200, confidence=2),
                    objectPerceptionQuality=3,
                ),
                stationIdInvolved=971391625,
                timeToCollision=1256,
                impactSection=ObjectFace.SIDE_LEFT_FRONT,
                estimatedBrakingDistance=1000,
            ),
        )
    )



def create_denm_message(
    station_id: int,
    lat: float,
    lon: float,
    heading: float = 0.0,
    speed_ms: float = 0.0,
    cause_code=None,
    time_to_collision: Optional[int] = None,
    estimated_braking_distance: Optional[int] = None,
    termination=None,
    validity_duration: int = 60,
    sequence_number: int = 1,
) -> DenmMessageV1:
    """
    Crea un DENM parametrizzato conforme ETSI EN 302 637-3.

    Args:
        station_id:                  ID stazione originante (vehicle ID CARLA)
        lat / lon:                   Posizione evento (WGS84)
        heading:                     Heading in gradi 0-360
        speed_ms:                    Velocita in m/s (convertita internamente in cm/s)
        cause_code:                  CauseCode ETSI (default: SLOW_VEHICLE)
        time_to_collision:           TTC in ms; None → preCrashContainer omesso
        estimated_braking_distance:  Distanza frenata in cm; None → N/A
        termination:                 Termination.CANCELLATION per cancellare un evento
        validity_duration:           Durata validita in secondi (default 60)
        sequence_number:             Numero sequenza ActionId (incrementare per update)

    Returns:
        DenmMessageV1 pronto per publish_its_message()
    """
    if cause_code is None:
        cause_code = CauseCode.SLOW_VEHICLE
    now_str = str(ItsTime())
    speed_val = int(speed_ms * 100)     # m/s → cm/s  (STEP convention)
    heading_val = int(heading * 10)     # deg → 0.1deg (STEP convention)

    pre_crash = None
    if time_to_collision is not None:
        pre_crash = PreCrashContainer(
            perceivedObject=PerceivedObject(
                objectId=None,
                measurementDeltaTime=2,
                position=ReferencePosition(
                    latitude=lat,
                    longitude=lon,
                    positionConfidence=PositionConfidenceEllipse(
                        semiMajorAxisLengthConfidence=500,
                        semiMinorAxisLengthConfidence=500,
                        semiMajorAxisOrientation=0,
                    ),
                    altitude=Altitude(value=0, confidence=None),
                ),
                speed=Speed(value=speed_val, confidence=2),
                heading=Heading(value=heading_val, confidence=2),
                objectDimensionZ=ObjectDimension(value=100, confidence=2),
                objectDimensionY=ObjectDimension(value=10, confidence=2),
                objectDimensionX=ObjectDimension(value=200, confidence=2),
                objectPerceptionQuality=3,
            ),
            stationIdInvolved=station_id,
            timeToCollision=time_to_collision,
            impactSection=ObjectFace.SIDE_LEFT_FRONT,
            estimatedBrakingDistance=estimated_braking_distance,
            estimatedCollisionLatitude=lat,      # ← richiesto da pydantic
            estimatedCollisionLongitude=lon,     # ← richiesto da pydantic
        )

    return DenmMessageV1(
        message=DenmDataV1(
            stationId=station_id,
            managementContainer=ManagementContainerDenm(
                actionId=ActionId(
                    originatingStationId=station_id,
                    sequenceNumber=sequence_number,
                ),
                detectionTime=now_str,
                referenceTime=now_str,
                termination=termination,
                eventPosition=ReferencePosition(
                    latitude=lat,
                    longitude=lon,
                    positionConfidence=PositionConfidenceEllipse(
                        semiMajorAxisLengthConfidence=500,
                        semiMinorAxisLengthConfidence=500,
                        semiMajorAxisOrientation=0,
                    ),
                    altitude=Altitude(value=0, confidence=None),
                ),
                awarenessDistance=199,
                trafficDirection=TrafficDirection.ALL_TRAFFIC_DIR,
                validityDuration=validity_duration,
                transmissionInterval=1000,
                stationType=TrafficParticipantType.PASSENGER_CAR,
                referenceStationId=station_id,
                referenceObjectId=0,
            ),
            situationContainer=SituationContainer(
                informationQuality=None,
                eventType=cause_code,
            ),
            locationContainer=LocationContainer(
                eventSpeed=Speed(value=speed_val, confidence=50),
                eventPositionHeading=Heading(value=heading_val, confidence=50),
                roadType=RoadType.URBAN_NO_STRUCT_SEP_TO_OPPOSITE_LANES,
            ),
            preCrashContainer=pre_crash,
        )
    )


def sync_test_publish_with_context_manager(logger: Logger) -> None:
    """
    Test the publishing functionality using a context manager.

    This function demonstrates the usage of the sync communication manager to publish ITS
    messages (CAM and DENM) and handle subscriptions. It showcases different message formats
    (JSON, ORM, DICT) and message types (CAM, DENM).

    Args:
        logger (Logger): Logger instance for recording operation details.

    Returns:
        None
    """
    logger.info(msg="Starting sync test with context manager")
    current_position = GpsLocation(
        time=ItsTime(),
        latitude=41.9792448,
        longitude=12.4923096,
        accuracy=500,
        speed=4525,
        heading=452,
    )
    address = GeoNetworkAddress(
        is_manual_configured=True,
        traffic_participant_type=TrafficParticipantType.PASSENGER_CAR,
        mac_id=123456789,
    )
    with sync_com_manager_create(logger=logger, station_id=None) as client:
        logger.info(msg=f"Client status: {client.connection_status}")
        client.subscribe(
            topics=[
                "v2x/cam/264421_4/g8/+/+/+/+/#",
                "v2x/denm/264421_4/g8/+/+/+/+/#",
            ],
            max_qos=MqttQoS.QOS_1,
            no_local=False, #True,
        )
        # CAM message as JSON
        client.publish_its_message(
            message=CamMessageV1().from_json(data=create_cam_json_message()),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # CAM message as ORM
        client.publish_its_message(
            message=create_cam_orm_message(),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # CAM message as DICT
        client.publish_its_message(
            message=CamMessageV1().from_dict(data=create_cam_dict_message()),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # DENM message as ORM
        client.publish_its_message(
            message=create_denm_message(),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # Simulate other operations
        time_sleep(2)
        # Extract messages received
        for message in client.its_messages_received:
            logger.info(msg=f"Message received: {message}")
    logger.info(msg=f"Client status: {client.connection_status}")


def sync_test_publish_without_context_manager(logger: Logger) -> None:
    """
    Tests publishing functionality without using context manager.

    This function demonstrates the basic usage of a synchronous communication manager
    for publishing ITS messages without using a context manager. It sets up a client,
    subscribes to specific topics, and handles cleanup properly.

    Args:
        logger (Logger): Logger instance for recording operation details and status.

    Returns:
        None

    Note:
        - The function creates and manages a SyncComManager instance
        - Subscribes to CAM and DENM topics
        - Contains commented-out examples of different message publishing methods
        - Properly handles client cleanup in the finally block
    """
    logger.info(msg="Starting sync test without context manager")
    current_position = GpsLocation(
        time=ItsTime(),
        latitude=41.9792448,
        longitude=12.4923096,
        accuracy=500,
        speed=4525,
        heading=452,
    )
    address = GeoNetworkAddress(
        is_manual_configured=True,
        traffic_participant_type=TrafficParticipantType.PASSENGER_CAR,
        mac_id=123456789,
    )
    client: Optional[SyncComManager] = None
    try:
        client = sync_com_manager_create(logger=logger, station_id=None)
        logger.info(msg=f"Client status: {client.connection_status}")
        # Start the client
        client.start()
        logger.info(msg=f"Client status: {client.connection_status}")
        # Subscribe to the topics
        client.subscribe(
            topics=["v2x/cam/264421_4/g8/+/+/+/+/#", 
                    "v2x/denm/264421_4/g8/+/+/+/+/#"
                    ],
            max_qos=MqttQoS.QOS_1,
            no_local=False, #True,
        )
        # CAM message as JSON
        client.publish_its_message(
            message=CamMessageV1().from_json(data=create_cam_json_message()),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # CAM message as ORM
        client.publish_its_message(
            message=create_cam_orm_message(),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # CAM message as DICT
        client.publish_its_message(
            message=CamMessageV1().from_dict(data=create_cam_dict_message()),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # DENM message as ORM
        client.publish_its_message(
            message=create_denm_message(),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # Simulate other operations
        time_sleep(2)
        # Extract messages received
        for message in client.its_messages_received:
            logger.info(msg=f"Message received: {message}")
    finally:
        if client is not None:
            # Start the client
            client.stop()
    logger.info(msg=f"Client status: {client.connection_status}")

class UtcFormatter(Formatter):
    """
    A logging formatter that converts time to UTC.

    This class extends the standard logging.Formatter class to ensure that
    all timestamps in log records are formatted in UTC time instead of local time.

    Attributes:
        converter (function): A function that converts a time tuple to UTC time,
            set to time.gmtime.
    """

    converter = time_gmtime



def sync_test_publish_with_context_manager_filter_station_id(logger: Logger) -> None:
    """
    Test publish functionality with context manager and station ID filtering.

    This test function demonstrates the publishing of various ITS (Intelligent
    Transport Systems) message formats using a synchronous communication manager
    with station ID filtering. It tests the following:

        1. Connection establishment with specific station ID
        2. Topic subscription for CAM and DENM messages
        3. Publishing ITS messages in different formats (JSON, ORM, Dict)
        4. Message reception verification

    Messages are published in the following formats:

        - CAM message from JSON
        - CAM message from ORM
        - CAM message from Dict
        - DENM message from ORM

    Args:
        logger (Logger): Logger instance for tracking test execution and results.

    Returns:
        None
    """
    logger.info(msg="Starting sync test with context manager and station Id filtering")
    current_position = GpsLocation(
        time=ItsTime(),
        latitude=41.9792448,
        longitude=12.4923096,
        accuracy=500,
        speed=4525,
        heading=452,
    )
    address = GeoNetworkAddress(
        is_manual_configured=True,
        traffic_participant_type=TrafficParticipantType.PASSENGER_CAR,
        mac_id=123456789,
    )
    with sync_com_manager_create(logger=logger, station_id=1254) as client:
        logger.info(msg=f"Client status: {client.connection_status}")
        time_sleep(1)
        client.subscribe(
            topics=[
                "v2x/cam/264421_4/g8/+/+/+/+/#",
                "v2x/denm/264421_4/g8/+/+/+/+/#",
            ],
            max_qos=MqttQoS.QOS_1,
            no_local=False,
        )
        # CAM message as JSON
        client.publish_its_message(
            message=CamMessageV1().from_json(data=create_cam_json_message()),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # CAM message as ORM
        client.publish_its_message(
            message=create_cam_orm_message(),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # CAM message as DICT
        client.publish_its_message(
            message=CamMessageV1().from_dict(data=create_cam_dict_message()),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # DENM message as ORM
        client.publish_its_message(
            message=create_denm_message(),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        # Simulate other operations
        time_sleep(2)
        # Extract messages received
        for message in client.its_messages_received:
            logger.info(msg=f"Message received: {message}")
    logger.info(msg=f"Client status: {client.connection_status}")



import uuid
import time


        




class CarlaStepClient:
#    def __init__(self, log_file="step_publish.log"):
    def __init__(self, log_file="step_publish.log", station_id=None):

        logger = getLogger(name=__name__)
        logger.setLevel(level=logging.DEBUG) #[PA]
        #logger.setLevel(level=logging.INFO)
       # ✅ GENERA CLIENT ID UNIVOCO
        if station_id is None:
            station_id = int(time.time() * 1000) % 1000000
        
        unique_client_id = f"carla-v2x-{uuid.uuid4().hex[:8]}-{station_id}"
        print(f"✅ Client ID univoco: {unique_client_id}")


        channel = FileHandler(
            filename=log_file,
            mode="w",
            encoding="utf-8",
            delay=False,
            errors=None,
        )
        #verificare la gestione del livello di logging client vs cahnnel [PA]
        channel.setLevel(level=logging.DEBUG)
        formatter = UtcFormatter(
            fmt="%(asctime)s.%(msecs)03d [%(levelname)s][pid=%(process)d][thread=%(threadName)s][module=%(module)s][function=%(funcName)s]: %(message)s",  # pylint: disable=line-too-long
            datefmt="%Y-%m-%d %H:%M:%S",
        )
        channel.setFormatter(fmt=formatter)
        logger.addHandler(hdlr=channel)
        logger.propagate = False # Add by a.solinas, to not print on terminal step manager loggings during sim
        #self.client = sync_com_manager_create(logger=logger, station_id=station_id)  [PA]
        self.client = sync_com_manager_create(logger=logger,station_id=station_id)
        self.client.start()
        self.logger = logger
        time_sleep(1)
        self.client.subscribe(
            topics=[
                "v2x/cam/264421_4/g8/+/+/+/+/#",
                "v2x/denm/264421_4/g8/+/+/+/+/#",
            ],
            max_qos=MqttQoS.QOS_0,
            no_local=False,                     #no echo, altrimenti False
        )



    def test_publish_cam(self, lat=45.4642, lon=9.1900, speed=12.0, station_id=999999):
        """TEST CAM - FIXED Speed int (cm/s)."""
        current_pos = GpsLocation(
            time=ItsTime(), 
            latitude=lat, 
            longitude=lon, 
            accuracy=1, 
            speed=int(speed * 100),  # 🔥 12.0 m/s → 1200 cm/s (INT!)
            heading=int(0 * 10)      # 0.0° → 0 (int, decimi di grado)
        )
        
        address = GeoNetworkAddress(
            is_manual_configured=True,
            traffic_participant_type=TrafficParticipantType.PASSENGER_CAR,
            mac_id=station_id
        )
        
        message = create_cam_orm_message(current_pos, station_id, TrafficParticipantType.PASSENGER_CAR)
        
        self.client.publish_its_message(
            message=message,
            address=address,
            current_position=current_pos,
            is_station_mobile=True
        )
        print(f"🔥 INVIO TEST CAM da {station_id}: ({lat:.6f}, {lon:.6f}), speed={speed} m/s")
        print(f"   📤 Pubblicato su: v2x/cam/264421_4/g8/{station_id}/...")




    def get_cam_messages(self, clear_after: bool = True) -> List[Dict[str, Any]]:
        """Estrae CAM da STEP. ✅ CORRETTA col tuo DEBUG."""
        cams = []
        
        print(f"CAM Estrae e parsifica CAM da STEP (MQTT). Ritorna lista dict.")

        # FIXED: generator → list
        #messages = list(self.client.its_messages_received)
        

        # 🔥 DEBUG: Verifica STEP
        print(f"🔍 DEBUG - itsmessagesreceived type: {type(self.client.its_messages_received)}")
        print(f"🔍 DEBUG - itsmessagesreceived len: {len(list(self.client.its_messages_received))}")
        #print(f"🔍 DEBUG - itsmessagesreceived len: {len(self.client.its_messages_received)}")
        print(f"🔍 DEBUG - client status: {self.client.connection_status}")
        print(f"🔍 DEBUG - subscribed topics: {getattr(self.client, '_subscribe_topics', 'N/A')}")
        
        # Process received messages
        for message in self.client.its_messages_received:
            print(f"Received message: {message}")


        messages = list(self.client.its_messages_received)
        print(f"🔍 DEBUG - messages dopo list(): {len(messages)} elementi")
        
        if len(messages) == 0:
            print("❌ NESSUN MESSAGGIO RICEVUTO - controlla:")
            print("  1. STEP connesso a MQTT?")
            print("  2. Qualcuno pubblica su v2x/cam/* ?")
            print("  3. Topics corretti? (v2xcam2644214#g8)")
            return []
    




        for msg in messages[:]:  # Copia per remove sicuro
            print(f"DEBUG msg: {type(msg)}, msg.message: {type(msg.message) if hasattr(msg, 'message') else 'NOPE'}")
            print(f"DEBUG attrs: {dir(msg.message) if hasattr(msg, 'message') else 'N/A'}")
            try:
                # Dal DEBUG: msg = CamMessageV1, msg.message = CamDataV1
                if hasattr(msg, 'message') and isinstance(msg.message, CamDataV1):
                    cam = msg.message  # ← CamDataV1 (NON cam_data!)
                    
                    cams.append({
                        'station_id': cam.stationInfo.station_id,           # ✅ stationInfo
                        'lat': cam.position.latitude,                       # ✅ position.latitude
                        'lon': cam.position.longitude,
                        'speed': cam.speed.value if cam.speed else 0,       # ✅ speed.value
                        'heading': cam.heading.value if cam.heading else 0,  # ✅ heading.value
                        'timestamp': str(cam.creationTime),                 # ✅ creationTime
                        'drive_dir': cam.driveDirection.value if cam.driveDirection else 'unknown',
                        'accel_long': cam.longitudinalAcceleration.value if cam.longitudinalAcceleration else 0,
                        'curvature': cam.curvature.value if cam.curvature else 0,
                        'yaw_rate': cam.yawRate.value if cam.yawRate else 0,
                        'raw': msg  # Messaggio STEP completo
                    })
                    
                    # LOG visibile
                    self.logger.info(f"CAM da {cam.stationInfo.station_id}: "
                                f"({cam.position.latitude:.6f}, {cam.position.longitude:.6f}), "
                                f"speed={cam.speed.value if cam.speed else 0:.1f}")
                    print(f"*** CAM [{len(cams)}] da {cam.stationInfo.station_id}: "
                        f"({cam.position.latitude:.6f}, {cam.position.longitude:.6f}), "
                        f"speed={cam.speed.value if cam.speed else 0:.1f} m/s")
                    
                    if clear_after:
                        # Rimuovi dalla coda originale
                        try:
                            self.client.itsmessagesreceived.remove(msg)
                        except ValueError:
                            pass  # Già rimosso
                            
            except Exception as e:
                self.logger.warning(f"Errore parsing CAM {msg}: {e}")
                continue
        
        self.logger.info(f"Trovati {len(cams)} CAM messages")
        return cams






    def get_cam_messages1(self, clear_after: bool = True) -> List[Dict[str, Any]]:
        """Estrae e parsifica CAM da STEP (MQTT). Ritorna lista dict."""
        cams = []

        print(f"CAM Estrae e parsifica CAM da STEP (MQTT). Ritorna lista dict.")
        
        #messages = self.client.its_messages_received
        messages = list(self.client.its_messages_received)
 




        for msg in messages[:]:  # Copia per modificare
            print(f"DEBUG msg: {type(msg)}, msg.message: {type(msg.message) if hasattr(msg, 'message') else 'NOPE'}")
            print(f"DEBUG attrs: {dir(msg.message) if hasattr(msg, 'message') else 'N/A'}")
            if hasattr(msg.message, 'CamDataV1'):
                print(f"cam_data OK: {dir(msg.message.CamDataV1)}")

            if hasattr(msg.message, 'CamDataV1'):  # È un CAM
                cam = msg.message.CamDataV1
                cams.append({
                    'station_id': cam.station_info.station_id,
                    'lat': cam.position.latitude,
                    'lon': cam.position.longitude,
                    'speed': cam.speed.value if cam.speed else 0,
                    'heading': cam.heading.value if cam.heading else 0,
                    'timestamp': str(cam.creation_time),
                    'raw': msg  # Messaggio STEP completo
                })
                self.logger.info(f"CAM da {cam.station_info.station_id}: ({cam.position.latitude:.6f}, {cam.position.longitude:.6f})")
                print(f"*********************************************************************************************************************************************************CAM da {cam.station_info.station_id}: ({cam.position.latitude:.6f}, {cam.position.longitude:.6f})")
                if clear_after:
                    messages.remove(msg)
        return cams  

    def get_messages(self, clear_after: bool = True) -> List[Dict[str, Any]]:
        """Estrae e parsifica DENM da STEP. Ritorna lista eventi."""
        message = None
        cam = None

        for message in self.client.its_messages_received:
            self.logger.info("Message received Pietro: %s", message)
            print(f"get_messagesget_messagesget_messagesget_messages*********************************************************************************************************************************************************)")
            if not hasattr(message.message, 'CamDataV1'):
                print(f"cam_data OK: {dir(message.message)}")
                print(f"cam_data OK: {dir(message.message)}")
        return message  
#-> List[Dict[str, Any]]:


    def get_cam_messages_list1(self, clear_after: bool = True) -> List[Dict[str, Any]]:
        """Estrae CAM da STEP. Ritorna LISTA DICT."""
        cams = []  # ← LISTA, non singolo event!
        
        messages = list(self.client.its_messages_received)


        for msg in messages:
            try:
                if hasattr(msg, 'message') and hasattr(msg.message, 'creationTime'):
                    cam = msg.message  # Oggetto CamDataV1
                    
                    event = {  # ← DICT singolo
                        'type': 'CAM',
                        'station_id': cam.stationInfo.stationId,
                        'station_type': str(cam.stationInfo.stationType),
                        'lat': cam.position.latitude,
                        'lon': cam.position.longitude,
                        'altitude': cam.position.altitude.value if cam.position.altitude else 0,
                        'heading': cam.heading.value if cam.heading else 0,
                        'speed_ms': cam.speed.value / 100 if cam.speed else 0,
                        'drive_dir': str(cam.driveDirection) if cam.driveDirection else '',
                        'vehicle_length': cam.vehicleLength.vehicleLengthValue,
                        'vehicle_width': cam.vehicleWidth,
                        'timestamp': str(cam.creationTime),
                        'raw': str(msg)
                    }
                    
                    cams.append(event)  # ← AGGIUNGI alla lista!
                    #print(f"[CAM] ID={event['station_id']} @ {event['lat']:.6f},{event['lon']:.6f}")
                
                if clear_after:
                    self.client.its_messages_received.remove(msg)
            
            except Exception as e:
                self.logger.warning(f"Parse error: {e}")
        
        return cams  # ← RITORNA LISTA!



    def get_cam_messages(self, clear_after: bool = True) -> Dict[str, Any]: 
        message = self.client.its_messages_received
        cam = None
        event = [str, Any]
        
        print(f"DEBUG - itsmessagesreceived len: {(message)}")
        
        for msg in message:
            self.logger.info("Message received: %s", msg)
            
            try:
                # Parsing CAM dal tuo log
                if hasattr(msg, 'message') and hasattr(msg.message, 'creationTime'):
                    cam = msg.message  # Direttamente su msg.message
                    event = {
                        'type': 'CAM',
                        'station_id': cam.stationInfo.stationId,  # 255
                        'station_type': str(cam.stationInfo.stationType),  # 'passengerCar'
                        'lat': cam.position.latitude,  # 41.5007684
                        'lon': cam.position.longitude,  # 2.0908633
                        'altitude': cam.position.altitude.value if cam.position.altitude else 0,
                        'heading': cam.heading.value if cam.heading else 0,  # 65 (0.1°)
                        'speed_ms': cam.speed.value / 100 if cam.speed else 0,  # 794 → 7.94 m/s (STEP: cm/s!)
                        'drive_dir': str(cam.driveDirection),
                        'vehicle_length': cam.vehicleLength.vehicleLengthValue,  # 30 (0.1m)
                        'vehicle_width': cam.vehicleWidth,  # 10 (0.1m)
                        'timestamp': str(cam.creationTime),  # 2026-02-12T15:26:33.516Z
                        'raw': str(msg)
                    }
                    
                    print(f"[CAM PARSED: ID={event['station_id']} @ {event['lat']:.6f},{event['lon']:.6f} speed={event['speed_ms']:.2f}m/s")
                
                if clear_after:
                    self.client.its_messages_received.remove(msg)
            
            except Exception as e:
                self.logger.warning("Parse error: %s", e)
        
        return event

    def get_cam_messages1(self, clear_after: bool = True) -> List[Dict[str, Any]]:
        """Estrae CAM → **DICT** (non oggetti STEP)."""
        cams = []  # Lista DICT
        messages_list = list(self.client.its_messages_received)
        
        for msg in messages_list[:]:
            try:
                # 🔥 STEP msg → DICT esplicito
                if hasattr(msg, 'message') and hasattr(msg.message, 'creationTime'):
                    cam_obj = msg.message  # <class 'type'>
                    
                    cam_dict = {  # ← CONVERTE in dict!
                        'type': 'CAM',
                        'station_id': cam_obj.stationInfo.stationId,
                        'station_type': str(cam_obj.stationInfo.stationType),
                        'lat': float(cam_obj.position.latitude),
                        'lon': float(cam_obj.position.longitude),
                        'altitude': cam_obj.position.altitude.value if cam_obj.position.altitude else 0,
                        'heading': cam_obj.heading.value if cam_obj.heading else 0,
                        'speed_ms': cam_obj.speed.value / 100.0 if cam_obj.speed else 0.0,
                        'drive_dir': str(cam_obj.driveDirection) if cam_obj.driveDirection else '',
                        'vehicle_length': cam_obj.vehicleLength.vehicleLengthValue / 10.0,
                        'vehicle_width': cam_obj.vehicleWidth / 10.0 if cam_obj.vehicleWidth else 0,
                        'timestamp': str(cam_obj.creationTime),
                        '_raw_obj': cam_obj  # Oggetto originale per debug
                    }
                    cams.append(cam_dict)  # ✅ DICT!
                    
                    print(f"[PARSE OK] ID={cam_dict['station_id']} → dict!")
                
                if clear_after:
                    self.client.its_messages_received.remove(msg)
                    
            except Exception as e:
                self.logger.warning("Parse fail %s: %s", msg, e)
        
        return cams  # Lista dict ✅


    def get_denm_messages(self, clear_after: bool = True) -> List[Dict[str, Any]]:
        """Estrae e parsifica DENM da STEP. Ritorna lista eventi."""
        denms = []

        #messages = self.client.its_messages_received
        messages = list(self.client.its_messages_received)

        for msg in messages[:]:
            if hasattr(msg.message, 'denm_data'):  # È un DENM
                denm = msg.message.denm_data
                denms.append({
                    'station_id': denm.station_id,
                    'event_type': denm.situation_container.event_type.value if denm.situation_container else 'unknown',
                    'lat': denm.management_container.event_position.latitude,
                    'lon': denm.management_container.event_position.longitude,
                    'severity': denm.situation_container.information_quality.value if denm.situation_container else None,
                    'timestamp': str(denm.management_container.detection_time),
                    'raw': msg
                })
                self.logger.info(f"DENM: {denm.situation_container.event_type.value if denm.situation_container else 'N/A'} da {denm.station_id}")
                if clear_after:
                    messages.remove(msg)
        return denms  

    def get_all_received(self, clear: bool = False) -> Dict[str, List]:
        """Ritorna TUTTI i CAM/DENM ricevuti (senza cancellare se clear=False)."""
        return {
            'cams': self.get_cam_messages(clear=False),
            'denms': self.get_denm_messages(clear=False),
            'total': len(self.client.its_messages_received)
        }



    def send_message(self, latitude: float, longitude: float, speed: float, heading: float, id: int):
        current_position = GpsLocation(
            time=ItsTime(),
            latitude=round(latitude,7), # Precision ~ 0.011m (1.1cm)
            longitude=round(longitude,7), # Precision ~ 0.011m (1.1cm)
            accuracy=1, # Precision ~ 0.011m (1.1cm)
            speed=int(speed*100), # converto a intero con precisione al centesimo di m/s
            heading=int(heading * 10), # converto a intero con precisione al decimo di grado
        )
        address = GeoNetworkAddress(
            is_manual_configured=True,
            traffic_participant_type=TrafficParticipantType.PASSENGER_CAR,
            mac_id=id, # Si può non utilizzare o inventare
        )
        # CAM message as ORM
        self.client.publish_its_message(
            message=create_cam_orm_message(current_position=current_position, station_id=id), # Aggiunto station_id [PA]
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )



    def send_denm_message(
        self,
        latitude: float,
        longitude: float,
        speed: float,
        heading: float,
        station_id: int,
        cause_code=None,
        time_to_collision: Optional[int] = None,
        estimated_braking_distance: Optional[int] = None,
        termination=None,
        validity_duration: int = 60,
        sequence_number: int = 1,
    ) -> None:
        """
        Invia un messaggio DENM ETSI tramite STEP/MQTT.

        Args:
            latitude / longitude:        Posizione evento (WGS84)
            speed:                       Velocita in m/s
            heading:                     Heading in gradi 0-360
            station_id:                  ID stazione CARLA del veicolo
            cause_code:                  CauseCode ETSI (default: SLOW_VEHICLE)
            time_to_collision:           TTC in ms (None → preCrashContainer omesso)
            estimated_braking_distance:  Distanza frenata in cm
            termination:                 Termination.CANCELLATION per cancellare evento
            validity_duration:           Durata validita in secondi
            sequence_number:             Numero sequenza (incrementare per update)

        Example::

            # Evento collisione imminente
            client.send_denm_message(
                latitude=45.4642, longitude=9.1900,
                speed=8.5, heading=90.0,
                station_id=1001,
                cause_code=CauseCode.COLLISION_RISK,
                time_to_collision=1500,
                estimated_braking_distance=800,
            )
            # Cancella evento precedente
            client.send_denm_message(
                latitude=45.4642, longitude=9.1900,
                speed=0.0, heading=0.0,
                station_id=1001,
                termination=Termination.CANCELLATION,
                sequence_number=2,
            )
        """
        current_position = GpsLocation(
            time=ItsTime(),
            latitude=round(latitude, 7),
            longitude=round(longitude, 7),
            accuracy=1,
            speed=int(speed * 100),
            heading=int(heading * 10),
        )
        address = GeoNetworkAddress(
            is_manual_configured=True,
            traffic_participant_type=TrafficParticipantType.PASSENGER_CAR,
            mac_id=station_id,
        )
        message = create_denm_message(
            station_id=station_id,
            lat=latitude,
            lon=longitude,
            heading=heading,
            speed_ms=speed,
            cause_code=cause_code,
            time_to_collision=time_to_collision,
            estimated_braking_distance=estimated_braking_distance,
            termination=termination,
            validity_duration=validity_duration,
            sequence_number=sequence_number,
        )
        self.client.publish_its_message(
            message=message,
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )
        self.logger.info(
            "DENM inviato — station=%s pos=(%.6f,%.6f) cause=%s ttc=%s term=%s",
            station_id, latitude, longitude, cause_code, time_to_collision, termination,
        )
        print(
            f"📡 DENM inviato da {station_id}: ({latitude:.6f}, {longitude:.6f}) "
            f"cause={cause_code} ttc={time_to_collision}ms seq={sequence_number}"
        )





    def get_received(self):
        return self.client.its_messages_received

if __name__ == "__main__":
    parser = argparse.ArgumentParser("tester")
    parser.add_argument("--listen", help="insted of sending a message, listed and print messages", action='store_true')
    args = parser.parse_args()
    client = CarlaStepClient("00_step_listen.log", station_id=None) if args.listen else CarlaStepClient("00_step_publish_test.log")
    if args.listen:
        while True:
            for message in client.get_received():
                print(f"Message received: {message}")
            time_sleep(2)
    else:
        for i in range(2):
            client.send_message(latitude=41.9792448+i, longitude=12.4923096+i, speed=4, heading=12, id=1)
        time_sleep(2)
        for message in client.get_received():
            print(f"Message received: {message}")

