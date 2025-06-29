o
    
�Tha  �                   @   s
  d dl m Z  d dlZi dd�dd�dd�d	d
�dd�dd�dd�dd�dd�dd�dd�dd�dd�dd�dd�d d!�d"d#�i d$d%�d&d'�d(d)�d*d+�d,d-�d.d/�d0d1�d2d3�d4d5�d6d7�d8d9�d:d;�d<d=�d>d?�d@dA�dBdC�dDdE��ZdFdG� ZdHdI� ZG dJdK� dK�ZdS )L�    )�datetimeN�rollz
Roll (rad)�pitchzPitch (rad)�yawz	Yaw (rad)�airspeedzAirspeed (m/s)�climbzClimb Rate (m/s)�altzAltitude (m)�latu   Latitude (°)�lonu   Longitude (°)�altitude�vxzVel X (m/s)�vyzVel Y (m/s)�vzzVel Z (m/s)�fix_typezGPS Fix�satellites_visiblez# Satellites�ephzHDOP (m)�voltagezVoltage (V)�voltageszVoltages (V)�current_batteryzBattery Current (A)�energy_consumedzEnergy Used (Wh)�rpmzESC RPM�temperatureu
   Temp (°C)�
servo1_rawzServo 1�
servo2_rawzServo 2�
servo3_rawzServo 3�
servo4_rawzServo 4�
servo5_rawzServo 5�
servo6_rawzServo 6�
servo7_rawzServo 7�
servo8_rawzServo 8�
servo9_rawzServo 9�commandz
Command ID�resultzResult Code�textzStatus Message�	timestamp�Timec                 C   s  || }i }|D ]l}t | |�rtt| |�}|dv r|d }nS|dv r&|d }nJ|dkr/|d9 }nA|dv r8|d9 }n8|d	krA|d9 }n/|d
krJ|d }n&|dkrVdd� |D �}n|dkr_|d9 }n|dkrh|d9 }n|dkrp|d9 }|||< qt�� }|�d�|d< |S )N)r	   r
   g    �cA)r   r   �     @�@�headingg�������?)�hor_velocity�ver_velocityr   r   r   g{�G�z�?r   �voltage_batteryr   c                 S   s   g | ]}|d  �qS )r&   � ��.0�vr+   r+   �B/home/kent/AmericanTenet/master folder/telemetry_tool/telemetry.py�
<listcomp>G   s    z$update_telemetry.<locals>.<listcomp>r   r   g      Y@r   z%H:%M:%Sr$   )�hasattr�getattrr   �now�strftime)�message�type�existing_type�field_names�msg_data�field�value�current_timer+   r+   r/   �update_telemetry*   s<   









�r=   c           	      C   s�   t � � }||  dkrW|} td|� d�� |�� D ]>\}}t|� d|� d�� |�� D ]+\}}t�||�}t|t�r?t|d�}nt|t�rKdd� |D �}td	|� d
|� �� q*qt| �S )N�   z
--- z ---z | �:�   c                 S   s(   g | ]}t |ttf�rt|d �n|�qS )�   )�
isinstance�int�float�roundr,   r+   r+   r/   r0   h   s   ( z$output_telemetry.<locals>.<listcomp>z  z: )	�time�print�items�FIELD_LABELS�getrB   rD   rE   �list)	�
print_time�phase�stored_datar<   �msg_type�data�keyr;   �labelr+   r+   r/   �output_telemetry\   s   

�rS   c                   @   s,   e Zd Zdd� Zdd� Zdd� Zdd� Zd	S )
�	Telemetryc                 C   s
   i | _ d S �N��master_storage��selfr+   r+   r/   �__init__m   s   
zTelemetry.__init__c                 C   s   t |||�| j|< d S rU   )r=   rW   )rY   �msgrO   �fields_by_typer+   r+   r/   �update_master_storagep   s   zTelemetry.update_master_storagec                    s   � fdd�|| D �S )Nc                    s"   i | ]}|� j v r|� j | �qS r+   rV   )r-   rO   rX   r+   r/   �
<dictcomp>t   s
    

�z3Telemetry.get_flight_phase_data.<locals>.<dictcomp>r+   )rY   �
curr_phase�types_by_phaser+   rX   r/   �get_flight_phase_datas   s   
�zTelemetry.get_flight_phase_datac                 C   s   t |||�S rU   )rS   )rY   rL   r_   �flight_phase_datar+   r+   r/   �output_phase_dataz   s   zTelemetry.output_phase_dataN)�__name__�
__module__�__qualname__rZ   r]   ra   rc   r+   r+   r+   r/   rT   l   s
    rT   )r   rF   rI   r=   rS   rT   r+   r+   r+   r/   �<module>   s�    ��������	�
���������������������� �!�"�%2