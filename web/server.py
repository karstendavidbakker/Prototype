from flask import Flask
app = Flask(__name__)

@app.route('/')
def hello_world():
    return 'Hello, World Vlammen Team!'

if __name__ == '__main__':
    app.run()


if __name__ == '__main__':
    app.run(host='0.0.0.0')

#This means that your server will accept incoming request from any source.
