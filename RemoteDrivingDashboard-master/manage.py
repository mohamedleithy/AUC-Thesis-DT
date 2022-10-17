#!/usr/bin/env python
import os
import sys
from src.local_cloud.Query_Search import select_data,create_DB

if __name__ == "__main__":
    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "web_project.settings")
    from django.core.management import execute_from_command_line
    #from django.conf import settings
    #from django.conf.urls.static import static
    execute_from_command_line(sys.argv)
