"""
WSGI config for web_project project.

It exposes the WSGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/1.11/howto/deployment/wsgi/
"""

import os

from django.core.wsgi import get_wsgi_application

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "Qyun.settings")

from socketio import Middleware
from apps.actuate.views import sio
from django.contrib.staticfiles.handlers import StaticFilesHandler

django_app = StaticFilesHandler(get_wsgi_application())
application = Middleware(sio, django_app)

#
import eventlet
import eventlet.wsgi
eventlet.wsgi.server(eventlet.listen(('', 8000)), application)