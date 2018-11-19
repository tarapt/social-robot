from flask import Flask, jsonify, abort

app = Flask(__name__)

users = [
    {
        "name": "Nicholas",
        "age": 42,
        "occupation": "Network Engineer"
    },
    {
        "name": "Elvin",
        "age": 32,
        "occupation": "Doctor"
    },
    {
        "name": "Jass",
        "age": 22,
        "occupation": "Web Developer"
    }
]

@app.route('/user/<string:name>', methods=['GET'])
def get(name):
    for user in users:
        if(name == user["name"]):
            return jsonify(user)
    abort(404)

if __name__ == '__main__':    
    app.run(debug=True)