def get_params():
    
    # ---Set Drone Parameters--- #
        robo_params = {"Mass":0.18, "I":0.00025, "L":0.086,
                       "MinF":0,"MaxF":2*0.18*9.81}

     #---Controller Parameters---#
        #controller_gains = {"kyp":20,"kyd":2.8/8,
        #              "kzp":1000,"kzd":60,
        #              "kPhip":3,
        #              "kPhid":0.005}

        controller_gains = {"kyp":29*0.8,"kyd":0.8*0.6*27,
                      "kzp":200,"kzd":2*200*0.05,
                      "kPhip":0.55,
                      "kPhid":0.0001}

        return robo_params, controller_gains
