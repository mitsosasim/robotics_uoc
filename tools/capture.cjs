// Capture the real replay UI from generated Python simulation output.
const {chromium} = require('playwright');
const path = require('node:path');
const fs = require('node:fs');
const http = require('node:http');
(async()=>{
 const root=path.resolve(__dirname,'..');
 const server=http.createServer((req,res)=>{
  const pathname=new URL(req.url,'http://localhost').pathname;
  const file=path.resolve(root,'.'+decodeURIComponent(pathname));
  if(!file.startsWith(root+path.sep)){res.writeHead(403);res.end();return}
  fs.readFile(file,(err,body)=>{if(err){res.writeHead(404);res.end();return}res.setHeader('Content-Type',file.endsWith('.json')?'application/json':'text/html');res.end(body)});
 });
 await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
 const port=server.address().port;
 const browser=await chromium.launch({headless:true,args:['--no-sandbox'],...(process.env.CHROMIUM_EXECUTABLE_PATH?{executablePath:process.env.CHROMIUM_EXECUTABLE_PATH}:{})});
 try{
  for(const n of [1,2,3]){
   const page=await browser.newPage({viewport:{width:1280,height:900},deviceScaleFactor:1});
   const errors=[];page.on('pageerror',e=>errors.push(e.message));
   await page.goto(`http://127.0.0.1:${port}/tools/viewer.html?assignment=${n}`);
   await page.waitForFunction(()=>window.replayReady===true);
   if(errors.length)throw Error(errors.join('\n'));
   const dest=path.resolve(__dirname,'../docs/assets',`assignment${n}.png`);
   fs.mkdirSync(path.dirname(dest),{recursive:true});
   await page.locator('.app').screenshot({path:dest});
   // Verify the controls affect the actual canvas, then restore the final frame.
   const before=await page.locator('#scene').screenshot();
   await page.locator('#restart').click();
   const after=await page.locator('#scene').screenshot();
   if(before.equals(after))throw Error('Replay frame did not change');
   await page.locator('#play').click();
   await page.waitForFunction(()=>Number(document.querySelector('#frame').value)>0);
   console.log('Captured and checked assignment '+n);
   await page.close();
  }
 }finally{await browser.close();await new Promise(resolve=>server.close(resolve))}
})().catch(e=>{console.error(e);process.exitCode=1});
