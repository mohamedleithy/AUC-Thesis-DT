#___python lib__
from django.shortcuts import resolve_url
import requests
import json
from datetime import datetime
import random
#___src___
from django.db import connections


def create_DB():
    cursor = connections["default"].cursor()
    # create_sense = """CREATE TABLE IF NOT EXISTS `sense_table` (
    #                     `time` datetime,
    #                     `ultrasonic_1` float,
    #                     `ultrasonic_2` float,
    #                     `ultrasonic_3` float,
    #                     `temperature` float,
    #                     `speed` float,
    #                     `lat` float,
    #                     `lng` float,
    #                     `id` int NOT NULL AUTO_INCREMENT,
    #                     PRIMARY KEY (id) 
    #                     )
    #                     """
    # cursor.execute(create_sense)
    # create_actuate =  """CREATE TABLE IF NOT EXISTS `actuate_table` (
    #                     `time` datetime,
    #                     `signal_name` varchar(20),
    #                     `signal_value` float,
    #                     `id` int NOT NULL AUTO_INCREMENT,
    #                     PRIMARY KEY (id),
    #                     UNIQUE (signal_name) 
    #                     )
    #                     """
    # cursor.execute(create_actuate)
    # date = datetime.now()
    # create_historical =  """CREATE TABLE IF NOT EXISTS `historical_actuate` (
    #                 `time` datetime,
    #                 `control_speed` float default NULL,
    #                 `steering_angle` float default NULL,
    #                 `direction` int default NULL,
    #                 `id` int NOT NULL AUTO_INCREMENT,
    #                 PRIMARY KEY (id) 
    #                 )
    #                 """
    # cursor.execute(create_historical)

    # sql_statement = """INSERT IGNORE INTO `actuate_table`
    #                     (`signal_name`,
    #                      `signal_value`,
    #                     `time`
    #                     )
    #                     VALUES
    #                     (%s,%s,%s), (%s,%s,%s), (%s,%s,%s);
    #                 """
    # cursor.execute(sql_statement,['control_speed',0,date,'steering_angle',0,date,'direction',0,date])
    


# def select_data(ranges_dict):
#     cursor = connections["default"].cursor()
#     data = []
#     sql_statement = """select * from sense_table"""
#     if bool(ranges_dict) :
#         sql_statement += """ where """
#         if ranges_dict["FromDate"] and ranges_dict["ToDate"]:
#             sql_statement += """time BETWEEN '""" + ranges_dict["FromDate"]+"""' AND '""" + ranges_dict["ToDate"]+"""' """
#     else:
#         sql_statement +=""" ORDER BY ID DESC LIMIT 40"""
        
#     try:
#         cursor.execute(sql_statement)
#         columns = [column[0] for column in cursor.description]
#         for row in cursor.fetchall():
#             data.append(dict(zip(columns, row)))
#     except Exception as e:
#         print(e)    
#     return data



def select_data(ranges_dict):

    # database connection
    cursor = connections["default"].cursor()

    # data to be sent to frontend
    historical_data = {}

    # initializing sql_statement to be executed
    sql_statement = ""

    
    # check if dates are entered 
    if bool(ranges_dict) :
        # get data in time range specified
        sql_statement = """ where """
        if ranges_dict["FromDate"] and ranges_dict["ToDate"]:
            sql_statement += """time BETWEEN '""" + ranges_dict["FromDate"]+"""' AND '""" + ranges_dict["ToDate"]+"""' """
    
    # to handle if date range is null, return first 40 values
    # else:
    #     sql_statement =""" ORDER BY ID DESC LIMIT 40"""


    # fetching all tables
    cursor.execute("Show tables;")
    all_tables = cursor.fetchall()

    # conversting tuple to list
    all_tables = ([a_tuple[0] for a_tuple in all_tables])

    # neglecting django.migration table
    all_tables.remove("django_migrations")

    print(all_tables)

    # get time values from the first table in database
    cursor.execute(f"""SELECT time FROM {all_tables[1]}""" + sql_statement + ";")
    time = cursor.fetchall()
    time = ([a_tuple[0] for a_tuple in time])
    
    # add time data
    historical_data["time"] = time
    
    # print(historical_data)    
    # print(time)


    # executing sql statements 
    for table in all_tables:
        # print(table)
        # select value from tables 
        cursor.execute(f"""SELECT value FROM {table}""" + sql_statement)
        sensors_values = cursor.fetchall()

        # add all sensors values 
        historical_data[table] = ([a_tuple[0] for a_tuple in sensors_values])
    

    print(historical_data)

    # try:
    # #     cursor.execute(sql_statement)
    #       columns = [column[0] for column in cursor.description]
    # #     for row in cursor.fetchall():
    # #         data.append(dict(zip(columns, row)))
    # except Exception as e:
    #      print(e)   

    return historical_data


def actuate_DB(sig_val, sig_name):
    cursor = connections["default"].cursor()
    date = datetime.now()
    if sig_val:
        sql_statement = """UPDATE actuate_table
                        SET signal_value = %s, time = %s
                        WHERE signal_name = %s
                        """
        cursor.execute(sql_statement,[sig_val,date,sig_name])
        sql_statement1 = """INSERT INTO historical_actuate (time,"""+sig_name+""") VALUES (%s,%s);"""
        cursor.execute(sql_statement1,[date,sig_val])

