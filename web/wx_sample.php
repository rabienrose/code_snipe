<?php
define("TOKEN", "chamo");
$wechatObj = new wechatCallbackapiTest();
$wechatObj->responseMsg();

class wechatCallbackapiTest
{
	public function valid(){
    $echoStr = $_GET["echostr"];

    //valid signature , option
    if($this->checkSignature()){
    	echo $echoStr;
    	exit;
    }
  }

  public function responseMsg(){
		$postStr = $GLOBALS["HTTP_RAW_POST_DATA"];
		if (empty($postStr)){
			exit;
		}

    libxml_disable_entity_loader(true);
  	$postObj = simplexml_load_string($postStr, 'SimpleXMLElement', LIBXML_NOCDATA);
    $fromUsername = $postObj->FromUserName;
    $toUsername = $postObj->ToUserName;
    $keyword = trim($postObj->Content);
    $time = time();
    $textTpl = "<xml>
								<ToUserName><![CDATA[%s]]></ToUserName>
								<FromUserName><![CDATA[%s]]></FromUserName>
								<CreateTime>%s</CreateTime>
								<MsgType><![CDATA[%s]]></MsgType>
								<Content><![CDATA[%s]]></Content>
								<FuncFlag>0</FuncFlag>
								</xml>";
		if(empty( $keyword )){
			exit;
		}
		
		$msgType = "text";
  	$contentStr = $keyword;
  	
		while($keyword == "create"){
			$con = mysql_connect("bdm246008380.my3w.com","bdm246008380","la009296");
			if (!$con){
		  	$contentStr ="fail connect!!";
		  	break;
		  }
		  if (!mysql_select_db("bdm246008380_db", $con)){
		  	$contentStr ="fail select!!";
		  	break;
		  }
//			$sql = "CREATE TABLE Persons 
//							(
//							FirstName varchar(15),
//							LastName varchar(15),
//							Age int
//							)";
//			if(!mysql_query($sql,$con)){
//				$contentStr ="fail query!!";
//		  	break;
//			}
//
			$sql="INSERT INTO Persons (FirstName, LastName, Age)
						VALUES ('chamo','wang','12')";
			if (!mysql_query($sql,$con)){
			  $contentStr ="fail query!!";
		  	break;
		  }
			mysql_close($con);
		  $contentStr ="create success.";
		  break;
		}
  	$resultStr = sprintf($textTpl, $fromUsername, $toUsername, $time, $msgType, $contentStr);
  	echo $resultStr;
  }
		
	private function checkSignature(){
    // you must define TOKEN by yourself
    if (!defined("TOKEN")) {
        throw new Exception('TOKEN is not defined!');
    }
    
    $signature = $_GET["signature"];
    $timestamp = $_GET["timestamp"];
    $nonce = $_GET["nonce"];
        		
		$token = TOKEN;
		$tmpArr = array($token, $timestamp, $nonce);
        // use SORT_STRING rule
		sort($tmpArr, SORT_STRING);
		$tmpStr = implode( $tmpArr );
		$tmpStr = sha1( $tmpStr );
		
		if( $tmpStr == $signature ){
			return true;
		}else{
			return false;
		}
	}
}

?>