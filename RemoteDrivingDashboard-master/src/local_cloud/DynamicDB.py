from django.db import connections

# Class Singleton to make sure only one instance is created from the DB Table
class Singleton:
    __instance = None
    @staticmethod
    def getInstance():
        if Singleton.__instance == None:
            Singleton()
            return Singleton.__instance
    def __init__(self):
        if Singleton.__instance != None:
            raise Exception("This class is a singleton!")
        else:
            Singleton.__instance = self

    def create_Table(self, name):
     cursor = connections["default"].cursor()
     create_sense = """CREATE TABLE IF NOT EXISTS """ + name + """ (
                         `time` datetime,
                         `value` float,
                         `id` int NOT NULL AUTO_INCREMENT,
                         PRIMARY KEY (id) 
                         )
                         """
     cursor.execute(create_sense)





