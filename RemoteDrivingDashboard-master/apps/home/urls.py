from apps.home import views
from django.conf.urls import url

urlpatterns = [
   url(r'^$',views.main,name='main')
]
