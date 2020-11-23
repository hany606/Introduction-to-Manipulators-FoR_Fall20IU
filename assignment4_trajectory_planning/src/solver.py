from utils import *

def parser_expr(T_str):
    shift = 7
    num_mat = len(T_str)//shift
    # print(num_mat, len(T_str)/shift)
    expr = ""
    for i in range(num_mat):
        if(i == 0): # T_base
            expr += "T_base"
        elif(i == num_mat-1): # T_tool
            expr += "T_tool"
        else:
            transformation = ""
            if(T_str[i*shift] == 'R'):
                if(T_str[i*shift+1] == 'x'):
                    transformation = f"mat(rotation_x({T_str[i*shift+3]}[{T_str[i*shift+4]}]))"
                elif(T_str[i*shift+1] == 'y'):
                    transformation = f"mat(rotation_y({T_str[i*shift+3]}[{T_str[i*shift+4]}]))"
                elif(T_str[i*shift+1] == 'z'):
                    transformation = f"mat(rotation_z({T_str[i*shift+3]}[{T_str[i*shift+4]}]))"
            elif(T_str[i*shift] == 'T'):
                if(T_str[i*shift+1] == 'x'):
                    transformation = f"mat(translation_x({T_str[i*shift+3]}[{T_str[i*shift+4]}]))"
                elif(T_str[i*shift+1] == 'y'):
                    transformation = f"mat(translation_y({T_str[i*shift+3]}[{T_str[i*shift+4]}]))"
                elif(T_str[i*shift+1] == 'z'):
                    transformation = f"mat(translation_z({T_str[i*shift+3]}[{T_str[i*shift+4]}]))"                    
            expr += transformation
        expr += "*"
    return expr[:-1]

def parser_FK(T_str):
    shift = 7
    num_mat = len(T_str)//shift
    # print(num_mat, len(T_str)/shift)
    expr = "[\n"
    for i in range(num_mat):
        if(i == 0): # T_base
            expr += "\tT_base_robot"
        elif(i == num_mat-1): # T_tool
            expr += "T_tool_robot"
        else:
            if(T_str[i*shift+3] == 'q'):
                expr = expr[:-1] + ",\n\t"
            transformation = ""
            if(T_str[i*shift] == 'R'):
                if(T_str[i*shift+1] == 'x'):
                    transformation = f"rotation_x({T_str[i*shift+3]}[{T_str[i*shift+4]}])"
                elif(T_str[i*shift+1] == 'y'):
                    transformation = f"rotation_y({T_str[i*shift+3]}[{T_str[i*shift+4]}])"
                elif(T_str[i*shift+1] == 'z'):
                    transformation = f"rotation_z({T_str[i*shift+3]}[{T_str[i*shift+4]}])"
            elif(T_str[i*shift] == 'T'):
                if(T_str[i*shift+1] == 'x'):
                    transformation = f"translation_x({T_str[i*shift+3]}[{T_str[i*shift+4]}])"
                elif(T_str[i*shift+1] == 'y'):
                    transformation = f"translation_y({T_str[i*shift+3]}[{T_str[i*shift+4]}])"
                elif(T_str[i*shift+1] == 'z'):
                    transformation = f"translation_z({T_str[i*shift+3]}[{T_str[i*shift+4]}])"                    
            expr += transformation
        expr += "@"
    expr = expr[:-1] + "\n]"
    return expr

def parser_IK(T_str):
    pass

def parser_Jacobian_ND(T_str):
    pass

if __name__ == "__main__":
    # Make transitions matrix in String form
    T_str = "Tb(00)*Rz(q0)*Tz(l0)*Tx(l1)*Ry(q1)*Tx(l2)*Ry(q2)*Tx(l3)*Rx(q3)*Tx(l4)*Ry(q4)*Rx(q5)*Tx(l5)*Tt(00)*"
    expr = parser_FK(T_str)
    print(expr)
    # Find FK
    # Make correct swapping
    # Get symbolic expressions
    # Get IK
    # Find jacobian using Numerical derivative
    # Find jacobian using 
