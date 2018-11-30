import os, sys, json

def main():
    out = open(r"C:\Projects\my_topograph\models\leaper.obj", "w")
    m = json.load(open(r"C:\Projects\my_topograph\models\leaper.txt"))
    
    vindex = 0
    for v in m["vertexPositions"]:
        if (vindex % 3) == 0:
           out.write("\nv ")
        out.write(str(v) + " ")
        vindex += 1
    out.write("\n")
    vindex = 0
    for v in m["triangles"]:
        if (vindex % 3) == 0:
           out.write("\nf ")
        out.write(str(v+1) + " ")   
        vindex += 1

if __name__ == "__main__":
    main()