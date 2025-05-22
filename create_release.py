import os
from shutil import rmtree
from urllib.request import Request, urlopen, urlretrieve
import json
try:
	import requests
except ImportError:
	print("you need to do 'python -m pip install requests'");
	exit(0);
import zipfile
import io
import time
from datetime import date
import tarfile
import subprocess

repo = "supermerill/SuperSlicer"
program_name = "SuperSlicer"
path_7zip = r"C:\Program Files\7-Zip\7z.exe"
# github classic personal access token, works with [gist, repo, workflow] permissions, should be something like "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"
github_auth_token = ""

def get_version():
	settings_stream = open("./version.inc", mode="r", encoding="utf-8");
	lines = settings_stream.read().splitlines();
	for line in lines:
		if("SLIC3R_VERSION_FULL" in line):
			elems = line.split("\"");
			return elems[1];
	return "";

found_win = False; 
found_linux = False; 
found_linux_appimage_gtk2 = False; 
found_linux_appimage_gtk3 = False; 
found_macos = False; 
found_macos_arm = False;
first_day = "";

# return True if he want to cntue new artifacts
def handle_artifact(json_artifact):
	global found_win
	global found_linux
	global found_linux_appimage_gtk2
	global found_linux_appimage_gtk3
	global found_macos
	global found_macos_arm
	global first_day
	
	if json_artifact["workflow_run"]["head_branch"] == "rc":
		if first_day == "":
			print("encounter the first rc at " + json_artifact["created_at"][:10]);
			first_day = json_artifact["created_at"][:10];
		if json_artifact["created_at"][:10] == first_day:
			print("Next artifact: " + json_artifact["name"]);
		else:
			print("End of rc artifacts. Closing");
			print("("+json_artifact["name"] + "  @ "+json_artifact["created_at"][:10]+")");
			return False;
		if json_artifact["name"] == "rc_win64" and not found_win:
			found_win = True;
			print("Found win64 artifact");
			print("ask for: "+json_artifact["archive_download_url"]);
			resp = requests.get(json_artifact["archive_download_url"], headers={'Authorization': 'token ' + github_auth_token,}, allow_redirects=True);
			print("win: " +str(resp));
			z = zipfile.ZipFile(io.BytesIO(resp.content))
			base_name = release_path+"/"+program_name+"_"+version+"_win64_"+date_str;
			z.extractall(base_name);
			try:
				ret = subprocess.check_output([path_7zip, "a", "-tzip", base_name+".zip", base_name]);
			except:
				print("Failed to zip the win directory, do it yourself");
		if json_artifact["name"] == "rc_"+program_name+"-macOS.dmg" and not found_macos:
			found_macos = True;
			print("Found macos-intel artifact");
			print("ask for: "+json_artifact["archive_download_url"]);
			resp = requests.get(json_artifact["archive_download_url"], headers={'Authorization': 'token ' + github_auth_token,}, allow_redirects=True);
			print("macos: " +str(resp));
			z = zipfile.ZipFile(io.BytesIO(resp.content));
			z.extractall(release_path);
			os.rename(release_path+"/"+program_name+"-macOS-intel.dmg", release_path+"/"+program_name+"_"+version+"_macos_"+date_str+".dmg");
		if json_artifact["name"] == "rc_"+program_name+"-macOS-arm.dmg" and not found_macos_arm:
			found_macos_arm = True;
			print("Found macos-arm artifact");
			print("ask for: "+json_artifact["archive_download_url"]);
			resp = requests.get(json_artifact["archive_download_url"], headers={'Authorization': 'token ' + github_auth_token,}, allow_redirects=True);
			print("macos-arm: " +str(resp));
			z = zipfile.ZipFile(io.BytesIO(resp.content));
			z.extractall(release_path);
			os.rename(release_path+"/"+program_name+"-macOS-arm.dmg", release_path+"/"+program_name+"_"+version+"_macos_arm_"+date_str+".dmg");
		if json_artifact["name"] == "rc_"+program_name+"-linux-x64-GTK2.AppImage" and not found_linux_appimage_gtk2:
			found_linux_appimage_gtk2 = True;
			print("Found ubuntu GTK2 artifact");
			print("ask for: "+json_artifact["archive_download_url"]);
			resp = requests.get(json_artifact["archive_download_url"], headers={'Authorization': 'token ' + github_auth_token,}, allow_redirects=True);
			print("appimage: " +str(resp));
			z = zipfile.ZipFile(io.BytesIO(resp.content));
			z.extractall(release_path);
			os.rename(release_path+"/"+program_name+"-linux-x64-GTK2.AppImage", release_path+"/"+program_name+"-ubuntu_22.04-gtk2-" + version + ".AppImage");
		if json_artifact["name"] == "rc_"+program_name+"-linux-x64-GTK3.AppImage" and not found_linux_appimage_gtk3:
			found_linux_appimage_gtk3 = True;
			print("Found ubuntu GTK3 artifact");
			print("ask for: "+json_artifact["archive_download_url"]);
			resp = requests.get(json_artifact["archive_download_url"], headers={'Authorization': 'token ' + github_auth_token,}, allow_redirects=True);
			print("appimage: " +str(resp));
			z = zipfile.ZipFile(io.BytesIO(resp.content));
			z.extractall(release_path);
			os.rename(release_path+"/"+program_name+"-linux-x64-GTK3.AppImage", release_path+"/"+program_name+"-ubuntu_22.04-" + version + ".AppImage");
		if json_artifact["name"] == "rc_"+program_name+"-linux-x64-GTK3.tgz" and not found_linux:
			found_linux = True;
			print("Found ubuntu GTK3 archive artifact");
			print("ask for: "+json_artifact["archive_download_url"]);
			resp = requests.get(json_artifact["archive_download_url"], headers={'Authorization': 'token ' + github_auth_token,}, allow_redirects=True);
			print("appimage: " +str(resp));
			z = zipfile.ZipFile(io.BytesIO(resp.content));
			z.extractall(release_path);
			base_path = release_path+"/"+program_name+"_" + version + "_linux64_" + date_str;
			os.rename(release_path+"/"+program_name+"-linux-x64-GTK3.tgz", base_path+".tgz");
			# try:
				# subprocess.check_output([path_7zip, "a", "-tzip", base_path+".tar.zip", base_path+".tar"]);
				# os.remove(base_path+".tar");
			# except:
				# with zipfile.ZipFile(base_path+"_bof.tar.zip", 'w') as myzip:
					# myzip.write(base_path+".tar");
	return  not (found_win and found_linux and found_linux_appimage_gtk2 and found_linux_appimage_gtk3 and found_macos and found_macos_arm);

date_str = date.today().strftime('%y%m%d');
version = get_version();
print("create release for: " + str(version));
release_path = "./build/release_"+str(version);
if(os.path.isdir(release_path)):
	rmtree(release_path);
	print("deleting old directory");
os.mkdir(release_path);
#urllib.urlretrieve ("https://api.github.com/repos/"+repo+"/actions/artifacts", release_path+"artifacts.json");
need_more = True
page = 1
while need_more and page < 10:
	with urlopen("https://api.github.com/repos/"+repo+"/actions/artifacts?page="+str(page)) as f:
		artifacts = json.loads(f.read().decode('utf-8'));
		print("there is "+ str(artifacts["total_count"])+ " artifacts in the repo");
		for entry in artifacts["artifacts"]:
			need_more = handle_artifact(entry);
			if not need_more:
				break;
	page = page + 1;

print("DONT FORGET TO PUSH YOUR MASTER");
