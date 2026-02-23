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
    ManagementContainer,
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
        station_id (Optional[int]): Optional station identifier for the MQTT client

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
        instance_name="step_lib_test_" + str(station_id),
        host="de-he-mn.mqtt.step.vodafone.com",
        port=8883,
        username="73f4f844-15c9-427b-8787-592b3c675d1d",
        password="a8d493c9-4214-4542-b8d7-ca1a137bfd73",
        station_id=station_id,
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
        tls_insecure=False,
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
                qos=0,
            ),
            PublishTopicParams(
                message_type=MessageId.DENM_PDU,
                header_topic="v2x/denm/264421_4",
                geohash_precision=8,
                qos=0,
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


def create_cam_orm_message() -> CamMessageV1:
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
                stationId=2802522858,
                stationType=TrafficParticipantType.PASSENGER_CAR,
            ),
            position=ReferencePosition(
                latitude=25.4344459,
                longitude=51.232539899999999,
                positionConfidence=PositionConfidenceEllipse(
                    semiMajorAxisLengthConfidence=203,
                    semiMinorAxisLengthConfidence=203,
                    semiMajorAxisOrientation=0,
                ),
                altitude=Altitude(value=3401, confidence=2),
            ),
            heading=Heading(
                value=0,
                confidence=None,
            ),
            speed=Speed(value=0, confidence=None),
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


def create_denm_orm_message() -> DenmMessageV1:
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
            managementContainer=ManagementContainer(
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
            no_local=True,
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
            message=create_denm_orm_message(),
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
            topics=["v2x/cam/264421_4/g8/+/+/+/+/#", "v2x/denm/264421_4/g8/+/+/+/+/#"],
            max_qos=MqttQoS.QOS_1,
            no_local=True,
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
            message=create_denm_orm_message(),
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
            message=create_denm_orm_message(),
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

class CarlaStepClient:
    def __init__(self, log_file="step_publish.log", station_id=1):
        print(station_id)
        logger = getLogger(name=__name__)
        logger.setLevel(level=logging.DEBUG)
        channel = FileHandler(
            filename=log_file,
            mode="w",
            encoding="utf-8",
            delay=False,
            errors=None,
        )
        channel.setLevel(level=logging.DEBUG)
        formatter = UtcFormatter(
            fmt="%(asctime)s.%(msecs)03d [%(levelname)s][pid=%(process)d][thread=%(threadName)s][module=%(module)s][function=%(funcName)s]: %(message)s",  # pylint: disable=line-too-long
            datefmt="%Y-%m-%d %H:%M:%S",
        )
        channel.setFormatter(fmt=formatter)
        logger.addHandler(hdlr=channel)
        self.client = sync_com_manager_create(logger=logger, station_id=station_id)
        self.client.start()
        self.logger = logger
        time_sleep(1)
        self.client.subscribe(
            topics=[
                "v2x/cam/264421_4/g8/+/+/+/+/#",
                "v2x/denm/264421_4/g8/+/+/+/+/#",
            ],
            max_qos=MqttQoS.QOS_1,
            no_local=False,
        )

    def send_message(self, latitude, longitude, speed, heading, id):
        current_position = GpsLocation(
            time=ItsTime(),
            latitude=latitude,
            longitude=longitude,
            accuracy=500,
            speed=speed,
            heading=int(heading),
        )
        address = GeoNetworkAddress(
            is_manual_configured=True,
            traffic_participant_type=TrafficParticipantType.PASSENGER_CAR,
            mac_id=id,
        )
        # CAM message as ORM
        self.client.publish_its_message(
            message=create_cam_orm_message(),
            address=address,
            current_position=current_position,
            is_station_mobile=True,
        )

    def get_received(self):
        return self.client.its_messages_received

if __name__ == "__main__":
    parser = argparse.ArgumentParser("tester")
    parser.add_argument("--listen", help="insted of sending a message, listed and print messages", action='store_true')
    args = parser.parse_args()
    client = CarlaStepClient("step_listen.log", station_id=2) if args.listen else CarlaStepClient()
    if args.listen:
        while True:
            for message in client.get_received():
                print(f"Message received: {message}")
            time_sleep(2)
    else:
        client.send_message(latitude=41.9792448, longitude=12.4923096, speed=4525, heading=452)
        time_sleep(2)
        for message in client.get_received():
            print(f"Message received: {message}")

