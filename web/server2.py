#import flask
from flask import Flask, request, render_template

#name site(created by flask) app
app = Flask(__name__)

#functions for the root folder
@app.route('/')
#display welcome message
def hello_world():
    return 'Hello, World Vlammen!'

@app.route('/home')
def home():
    return render_template('index.html')

@app.route('/gauge')
def gauge():
    return render_template('gauge.html')

#added to test
@app.route('/barchart')
def barchart():
    return render_template('barchart.html')


if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)
    #to give everyone from every pc access to web
    #subscribe by: http://localhost:5000/home
