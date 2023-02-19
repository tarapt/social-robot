import argparse
import pickle
from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC

FACIAL_ENCODINGS_PATH = "../trained_models/facial_encodings_python3.pickle"
LABEL_ENCODER_PATH = "../trained_models/faces_label_encoder.pickle"
TRAINED_MODEL_PATH = "../trained_models/faces_svm_model.pickle"

# load the face embeddings
print("[INFO] loading face embeddings...")
data = pickle.loads(open(FACIAL_ENCODINGS_PATH, "rb").read())

# encode the labels
print("[INFO] encoding labels...")
le = LabelEncoder()
labels = le.fit_transform(data["names"])

print("[INFO] training model...")
recognizer = SVC(C=1.0, kernel="linear", probability=True)
recognizer.fit(data["encodings"], labels)

# write the actual face recognition model to disk
f = open(TRAINED_MODEL_PATH, "wb")
f.write(pickle.dumps(recognizer))
f.close()
 
# write the label encoder to disk
f = open(LABEL_ENCODER_PATH, "wb")
f.write(pickle.dumps(le))
f.close()
