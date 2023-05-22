from django.shortcuts import redirect ,render

class SimpleMiddleware(object):
    def open_access_middleware(self, get_response):
        def middleware(request):
            print("middleware")
            response = get_response(request)
            response["Access-Control-Allow-Origin"] = "*"
            response["Access-Control-Allow-Headers"] = "*"
            return response
        return middleware

    
    def process_request(self, request):
        pages_without_session = ["/home/login/" ,
                                "/home/register/" ,
                                "/home/local/upload_local_profile/",
                                "/adminsVirtuaLab/login/"]

        page_name = str(request.path)

        app_name = page_name.split('/')[1]
        apps_without_session = [ "docs"]
        if not (page_name in pages_without_session or app_name in apps_without_session):
            if request.method != "POST":
                if request.path.split('/')[1] != "static" and request.path.split('/')[1] != "adminsVirtuaLab":
                    if ("client_session"        in request.session._session or
                        "local_client_session"  in request.session._session or
                        "client_register_username"  in request.session._session or
                        "register_post_items"  in request.session._session):
                        pass
                    else:
                        try:
                            if request.path.split('/')[2] == "sense_signals":
                                return redirect('/home/register/')
                        except :
                            pass
                        else:
                            return redirect('/home/login/') 

                if request.path.split('/')[1] == "adminsVirtuaLab":
                    if ("admin"        in request.session._session ):
                        pass
                    else:
                        return redirect('/adminsVirtuaLab/login/') 

    def process_response(self, request, response):
        #print("process_response")
        response["Access-Control-Allow-Origin"]="*"
        response["Access-Control-Allow-Headers"] = "*"
        return response

    
    def process_template_response(self, request, response):
        pages_without_session = ["home/login.html" ,
                                 "home/register.html" ,
                                 "home/upload_Local_profile.html",
                                 "home/deploying.html",
                                 "home/profile.html",
                                 "adminsVirtuaLab/login.html"]

        page_name = str(response.template_name)
        app_name = page_name.split('/')[0]
        apps_without_session = ["docs"]

             

        if page_name in pages_without_session or app_name in apps_without_session:
            try:
                response.context_data['session']=request.session._session
            except :
                try:
                    response.context_data={} #if no context data was alrerady defined
                    response.context_data['session']=request.session._session
                except:
                    pass
                
            return response
        else:
            if ("client_session"            in request.session._session or
                "local_client_session"      in request.session._session or
                "client_register_username"  in request.session._session or
                "register_post_items"  in request.session._session or
                "admin" in request.session._session):
                try:
                    response.context_data['session']=request.session._session
                except :
                    response.context_data={} #if no context data was alrerady defined
                    response.context_data['session']=request.session._session
                return response

            else:
                return response(request, 'home/login.html',{})  

        
        
 