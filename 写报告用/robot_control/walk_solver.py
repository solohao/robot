"""walk_solver.py -- offline solve and save trajectory"""
import argparse,os,time,sys
sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
def main():
 p=argparse.ArgumentParser()
 p.add_argument("--n",type=int,default=0)
 p.add_argument("--speed",type=float,default=0.9)
 p.add_argument("--output",type=str,default="walk_traj.npz")
 args=p.parse_args()
 from walk_main import save_trajectory
 nf=int(max(args.n,round(250/max(args.speed,0.1))))if args.n==0 else args.n
 msg="Speed: "+str(args.speed)+"x => "+str(nf)+" frames, output="+args.output
 print(msg)
 import time;t0=time.time()
 save_trajectory(args.output,nf)
 t1=time.time()
 print("Done in "+str(round(t1-t0,1))+"s")
 print("Run: python walk_runner.py --traj "+args.output)
if __name__=="__main__":main()
