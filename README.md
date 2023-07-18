# RMF_Compliant_Lift
A raspberry pi add-on kit that allows legacy elevators to be RMF compliant


ros2 topic pub -1 /lift_requests rmf_lift_msgs/LiftRequest "{lift_name: 'test_lift', session_id: 'test', request_type: 0, destination_floor: '0', door_state: 2}"
