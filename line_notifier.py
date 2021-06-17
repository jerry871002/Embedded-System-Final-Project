import requests
  
def line_message(msg):
    token2 = "zle1Bi1MvdQEW390Vf5WqM6tLdb0SaOksYONslBh3tM"
    headers2 =  {
        "Authorization": "Bearer " + token2, 
        "Content-Type" : "application/x-www-form-urlencoded"
   }
    payload = {'message': msg}
    r = requests.post("https://notify-api.line.me/api/notify", headers = headers2, params = payload)
    return r.status_code

def send_pic(msg, pic):
    token = "fMzINcJVBA2pp9iBqr2a3qPrqlVkWD0w1a0LZgTn1Tg"
    headers =  {
        "Authorization": "Bearer " + token
    }
    payload = {'message': msg}
    files = {'imageFile': open(pic, 'rb')}
    r = requests.post("https://notify-api.line.me/api/notify", headers = headers, params = payload, files = files)
    return r.status_code