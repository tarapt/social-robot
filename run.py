from eve import Eve
# from flask import jsonify

my_settings = {
    'DOMAIN': {'people': {}}
}

app = Eve(settings=my_settings)

@app.route('/hello')
def hello_world():
    # return jsonify({'x': 1, 'y': 2, 'z': 2})
    return 'hello world!'

if __name__ == '__main__':
    app.run(debug=True)