import requests
import numpy as np

CONFIDENCE_THRESHOLD = 0.00005

# find the closest point (Euclidean distance) from a list of points
def closest_node(node, nodes):
    nodes = np.asarray(nodes)
    dist_2 = np.sum((nodes - node)**2, axis=1)
    return np.argmin(dist_2)

def get_mean_face_position(faces):
    mean_faces = []
    for face in faces:
        result = []
        for point in face:
            if(point[2] > CONFIDENCE_THRESHOLD):
                result.append((point[0], point[1]))
        result = np.array(result)
        print(np.shape(result))
        if(np.shape(result)[0] > 0):
            mean_faces.append(np.mean(result, axis=0))
    print(mean_faces)
    rounded_mean_faces = np.rint(mean_faces)
    print(rounded_mean_faces)
    return rounded_mean_faces

POSE_DETECTION_URL = "http://localhost:5000/get_pose"
r = requests.get(POSE_DETECTION_URL)

pose = r.json()

# returns an array of points(x, y, score) for each person
bodies = np.array(pose['body'])
faces = np.array(pose['face'])
left_hand = np.array(pose['left_hand'])
right_hand = np.array(pose['right_hand'])

print("shapes...")
print (np.shape(bodies))
print (np.shape(faces))
print (np.shape(left_hand))
print (np.shape(right_hand))
print(".............")

mean_faces = get_mean_face_position(faces)

point = np.array([60, 320])
print(closest_node(point, mean_faces))

