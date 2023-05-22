# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.shortcuts import redirect #render ,
from django.template.response import TemplateResponse as render

import re
import socketio


# Implement a loop to create DB Tables
#dynamic_DB.create_Table("senior1")

async_mode = None
sio = socketio.Server(async_mode='eventlet',cors_allowed_origins=['http://127.0.0.1:8000','http://localhost:8000','http://0.0.0.0:8000'])



def main(request):
	return render(request, "home/dashboard_layout.html")



