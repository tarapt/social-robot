from eve import Eve

my_settings = {
    'DOMAIN': {'people': {}}
}

app = Eve(settings=my_settings)

@app.route('/hello')
def hello_world():
    return 'hello world!'

if __name__ == '__main__':
    app.run()