import os
from pymongo import MongoClient
from dotenv import load_dotenv
load_dotenv()


def get_database():

    # Provide the mongodb atlas url to connect python to mongodb using pymongo with .env
    CONNECTION_STRING = os.getenv("CONNECTION_STRING")

    # Create a connection using MongoClient. You can import MongoClient or use pymongo.MongoClient
    client = MongoClient(CONNECTION_STRING)
    print("Database connected")

    # Create or Read From the database for our example (we will use the same database throughout the tutorial
    db = client['parking']

    # get collection of mongoDB : status
    collection = db["status"]

    return collection


# This is added so that many files can reuse the function get_database()
if __name__ == "__main__":

    # Get the database
    collection = get_database()

    # Get Data from DB
    for item in collection.find():
        print(item)
