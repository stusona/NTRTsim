link_libraries(tgcreator controllers core)

add_executable(AppPassive500
    Passive500.cpp
    AppPassive500.cpp
) 

# To compile a controller, add a line like the
# following inside add_executable:
#    controllers/T6TensionController.cpp
